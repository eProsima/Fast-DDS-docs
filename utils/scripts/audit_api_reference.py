#!/usr/bin/env python3

"""Audit Fast DDS API reference pages against Doxygen XML.

This script compares the manually maintained ``docs/fastdds/api_reference`` RST
wrappers with the public API exported by a Fast-DDS checkout.

It is intentionally conservative:
- it audits exact targets for ``class``, ``struct``, ``enum``, ``typedef``,
  ``variable``, and ``define`` directives
- it skips strict validation of ``doxygenfunction`` targets because Breathe
  signatures are harder to normalize reliably from raw XML
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import xml.etree.ElementTree as ET

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
API_REFERENCE_ROOT = REPO_ROOT / "docs" / "fastdds" / "api_reference"
PUBLIC_API_PATH_PREFIXES = ("dds/", "rtps/", "statistics/")
PUBLIC_API_EXCLUDE_PATH_PREFIXES = ("utils/",)
PUBLIC_API_EXCLUDE_NAME_PATTERNS = (
    r"::iterator$",
    r"::const_iterator$",
    r"^std::hash<",
    r"traits<",
    r"::Context$",
    r"::Entry$",
    r"::FilterSampleInfo$",
    r"::IteratorIndex$",
    r"::AllocationConfiguration$",
    r"::UserWriteData$",
    r"::UserWriteDataPtr$",
)
DOXYGEN_KINDS = {
    "class": "doxygenclass",
    "struct": "doxygenstruct",
    "enum": "doxygenenum",
    "typedef": "doxygentypedef",
    "variable": "doxygenvariable",
    "define": "doxygendefine",
}


@dataclass(frozen=True)
class DocTarget:
    path: Path
    kind: str
    target: str

    @property
    def symbol_leaf(self) -> str:
        leaf = self.target.split("::")[-1]
        return re.sub(r"\(.*\)$", "", leaf)


@dataclass(frozen=True)
class ApiCompound:
    kind: str
    name: str
    file_path: str


@dataclass(frozen=True)
class Filters:
    include_path_prefixes: tuple[str, ...]
    exclude_path_prefixes: tuple[str, ...]
    exclude_name_patterns: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit docs/fastdds/api_reference against a Fast-DDS checkout or Doxygen XML."
    )
    parser.add_argument(
        "--fastdds-dir",
        type=Path,
        help="Path to a Fast-DDS checkout. If set, Doxygen XML is generated automatically.",
    )
    parser.add_argument(
        "--doxygen-xml-dir",
        type=Path,
        help="Path to an existing Doxygen XML directory. Skips Doxygen generation.",
    )
    parser.add_argument(
        "--api-reference-dir",
        type=Path,
        default=API_REFERENCE_ROOT,
        help=f"API reference root to audit. Default: {API_REFERENCE_ROOT}",
    )
    parser.add_argument(
        "--show",
        type=int,
        default=30,
        help="Maximum number of entries to print per section.",
    )
    parser.add_argument(
        "--profile",
        choices=("all-installed", "public-api"),
        default="public-api",
        help=(
            "Filtering profile for reported compounds. "
            "'public-api' keeps only high-level installed API candidates. "
            "'all-installed' reports everything found in installed public headers."
        ),
    )
    parser.add_argument(
        "--include-path-prefix",
        action="append",
        default=[],
        help="Only include compounds declared in headers under these prefixes, e.g. dds/ or rtps/.",
    )
    parser.add_argument(
        "--exclude-path-prefix",
        action="append",
        default=[],
        help="Exclude compounds declared in headers under these prefixes.",
    )
    parser.add_argument(
        "--exclude-name-regex",
        action="append",
        default=[],
        help="Exclude compounds whose qualified names match these regular expressions.",
    )
    parser.add_argument(
        "--report",
        choices=("summary", "compare", "json"),
        default="summary",
        help=(
            "'summary' prints the audit summary, "
            "'compare' prints only discrepancies between the present and expected wrapper sets, "
            "'json' emits the same data as JSON."
        ),
    )
    parser.add_argument(
        "--fail-on-findings",
        action="store_true",
        help=(
            "Exit with code 1 when extra wrapper files, missing wrapper targets, "
            "or wrapper files needing updates are detected."
        ),
    )
    args = parser.parse_args()

    if not args.fastdds_dir and not args.doxygen_xml_dir:
        parser.error("one of --fastdds-dir or --doxygen-xml-dir is required")

    return args


def parse_doc_targets(api_reference_dir: Path) -> list[DocTarget]:
    targets: list[DocTarget] = []
    for rst_path in sorted(api_reference_dir.rglob("*.rst")):
        content = rst_path.read_text(encoding="utf-8")
        for kind, target in re.findall(r"^\.\.\s+(doxygen\w+)::\s+(.+)$", content, re.M):
            targets.append(
                DocTarget(
                    path=rst_path.relative_to(api_reference_dir),
                    kind=kind,
                    target=target,
                )
            )
    return targets


def infer_file_kind(path: Path) -> str:
    stem = path.stem
    if stem in {"api_reference", "dds_pim", "rtps", "statistics", "transport", "log"}:
        return "index"
    if stem.endswith(("_toc", "_class")):
        return "index"
    if stem in {
        "core",
        "policy",
        "status",
        "condition",
        "domain",
        "publisher",
        "subscriber",
        "topic",
        "xtypes",
        "rpc",
        "participant",
        "reader",
        "writer",
        "history",
        "resources",
        "attributes",
        "data",
        "common",
        "transport_generic_interfaces",
        "chaining_transport",
        "shm_transport",
        "tcp_transport",
        "udp_transport",
        "colors",
    }:
        return "index"
    return "wrapper"


def generate_doxygen_xml(fastdds_dir: Path) -> Path:
    include_dir = fastdds_dir / "include" / "fastdds"
    if not include_dir.is_dir():
        raise SystemExit(f"Fast-DDS include directory not found: {include_dir}")

    if shutil.which("doxygen") is None:
        raise SystemExit("doxygen is required when using --fastdds-dir")

    tmpdir = Path(tempfile.mkdtemp(prefix="fastdds-api-audit-"))
    output_dir = tmpdir / "doxygen"
    doxyfile = tmpdir / "Doxyfile"
    doxyfile.write_text(
        textwrap.dedent(
            f"""\
            PROJECT_NAME = FastDDSAudit
            OUTPUT_DIRECTORY = {output_dir}
            INPUT = {include_dir}
            FILE_PATTERNS = *.h *.hpp *.hxx
            RECURSIVE = YES
            EXTRACT_ALL = YES
            HIDE_UNDOC_MEMBERS = NO
            HIDE_UNDOC_CLASSES = NO
            GENERATE_HTML = NO
            GENERATE_XML = YES
            XML_OUTPUT = xml
            QUIET = YES
            WARNINGS = NO
            ENABLE_PREPROCESSING = YES
            MACRO_EXPANSION = YES
            EXPAND_ONLY_PREDEF = NO
            PREDEFINED = HAVE_SECURITY
            TYPEDEF_HIDES_STRUCT = NO
            FULL_PATH_NAMES = YES
            STRIP_FROM_PATH = {fastdds_dir / 'include'}
            CLANG_ASSISTED_PARSING = NO
            """
        ),
        encoding="utf-8",
    )
    subprocess.run(["doxygen", str(doxyfile)], check=True)
    return output_dir / "xml"


def _is_public_file(file_path: str) -> bool:
    return (
        bool(file_path)
        and
        file_path.endswith((".h", ".hpp", ".hxx"))
        and "/detail/" not in file_path
        and "/config/" not in file_path
        and not file_path.endswith(".idl")
    )


def build_filters(args: argparse.Namespace) -> Filters:
    include_path_prefixes = tuple(args.include_path_prefix)
    exclude_path_prefixes = tuple(args.exclude_path_prefix)
    exclude_name_patterns = tuple(args.exclude_name_regex)

    if args.profile == "public-api":
        if not include_path_prefixes:
            include_path_prefixes = PUBLIC_API_PATH_PREFIXES
        if not exclude_path_prefixes:
            exclude_path_prefixes = PUBLIC_API_EXCLUDE_PATH_PREFIXES
        if not exclude_name_patterns:
            exclude_name_patterns = PUBLIC_API_EXCLUDE_NAME_PATTERNS

    return Filters(
        include_path_prefixes=include_path_prefixes,
        exclude_path_prefixes=exclude_path_prefixes,
        exclude_name_patterns=exclude_name_patterns,
    )


def compound_matches_filters(compound: ApiCompound, filters: Filters) -> bool:
    if filters.include_path_prefixes and not any(
        compound.file_path.startswith(prefix) for prefix in filters.include_path_prefixes
    ):
        return False
    if any(compound.file_path.startswith(prefix) for prefix in filters.exclude_path_prefixes):
        return False
    if any(re.search(pattern, compound.name) for pattern in filters.exclude_name_patterns):
        return False
    return True


def parse_doxygen_xml(xml_dir: Path) -> tuple[set[tuple[str, str]], list[ApiCompound]]:
    exact_targets: set[tuple[str, str]] = set()
    compounds: list[ApiCompound] = []

    for xml_path in xml_dir.glob("*.xml"):
        if xml_path.name == "index.xml":
            continue

        root = ET.parse(xml_path).getroot()
        compound = root.find(".//compounddef")
        if compound is None:
            continue

        compound_kind = compound.get("kind", "")
        compound_name = compound.findtext("compoundname", default="")
        compound_location = compound.find("location")
        compound_file_path = compound_location.get("file", "") if compound_location is not None else ""
        if (
            compound_kind in {"class", "struct"}
            and "eprosima::fastdds" in compound_name
            and _is_public_file(compound_file_path)
        ):
            exact_targets.add((DOXYGEN_KINDS[compound_kind], compound_name))
            compounds.append(ApiCompound(compound_kind, compound_name, compound_file_path))

        for member in compound.findall(".//memberdef"):
            member_kind = member.get("kind", "")
            if member_kind not in {"enum", "typedef", "variable", "define"}:
                continue
            if member.get("prot") not in (None, "public"):
                continue
            member_location = member.find("location")
            member_file_path = member_location.get("file", "") if member_location is not None else compound_file_path
            if not _is_public_file(member_file_path):
                continue

            if member_kind == "define":
                symbol = member.findtext("name", default="")
            else:
                symbol = member.findtext("qualifiedname", default="") or member.findtext("name", default="")

            if symbol:
                exact_targets.add((DOXYGEN_KINDS[member_kind], symbol))

    return exact_targets, compounds


def summarize(
    doc_targets: list[DocTarget],
    exact_targets: set[tuple[str, str]],
    api_compounds: list[ApiCompound],
    filters: Filters,
) -> tuple[list[Path], list[Path], list[ApiCompound]]:
    file_results: dict[Path, list[bool]] = defaultdict(list)

    documented_compounds = {
        target.target
        for target in doc_targets
        if target.kind in {"doxygenclass", "doxygenstruct"}
    }

    for target in doc_targets:
        if target.kind == "doxygenfunction":
            continue
        file_results[target.path].append((target.kind, target.target) in exact_targets)

    all_stale_files = sorted(path for path, hits in file_results.items() if hits and not any(hits))
    partial_stale_files = sorted(path for path, hits in file_results.items() if any(hits) and not all(hits))
    missing_compounds = sorted(
        (
            compound
            for compound in api_compounds
            if compound_matches_filters(compound, filters)
            if compound.name not in documented_compounds
        ),
        key=lambda compound: compound.name,
    )

    return all_stale_files, partial_stale_files, missing_compounds


def build_compare_lists(
    api_reference_dir: Path,
    doc_targets: list[DocTarget],
    all_stale_files: list[Path],
    partial_stale_files: list[Path],
    missing_compounds: list[ApiCompound],
) -> dict[str, list[str]]:
    documented_wrapper_files = sorted(
        {
            target.path
            for target in doc_targets
            if infer_file_kind(target.path) == "wrapper"
        }
    )
    stale_set = set(all_stale_files)
    should_exist_files = sorted(path for path in documented_wrapper_files if path not in stale_set)
    extra_present_files = sorted(path for path in documented_wrapper_files if path in stale_set)

    # When the filtered profile finds compounds with no wrapper page, keep them as
    # "missing targets" instead of inventing file names from inconsistent conventions.
    missing_targets = [compound.name for compound in missing_compounds]

    return {
        "present_files": [str(path) for path in documented_wrapper_files],
        "should_exist_files": [str(path) for path in should_exist_files],
        "extra_present_files": [str(path) for path in extra_present_files],
        "partial_stale_files": [str(path) for path in sorted(partial_stale_files)],
        "missing_targets": missing_targets,
    }


def print_section(title: str, entries: list[str], show: int) -> None:
    print(f"{title}: {len(entries)}")
    for entry in entries[:show]:
        print(f"  - {entry}")
    if len(entries) > show:
        print(f"  ... {len(entries) - show} more")


def print_compare(compare_data: dict[str, list[str]], show: int) -> None:
    print_section("Present wrapper files that should not exist", compare_data["extra_present_files"], show)
    print()
    print_section("Wrapper files that should exist but need updates", compare_data["partial_stale_files"], show)
    print()
    print_section("Public API targets with no wrapper file", compare_data["missing_targets"], show)


def has_findings(compare_data: dict[str, list[str]]) -> bool:
    return any(
        compare_data[key]
        for key in ("extra_present_files", "partial_stale_files", "missing_targets")
    )


def main() -> int:
    args = parse_args()
    filters = build_filters(args)

    xml_dir = args.doxygen_xml_dir
    if xml_dir is None:
        xml_dir = generate_doxygen_xml(args.fastdds_dir.resolve())
    elif not xml_dir.is_dir():
        raise SystemExit(f"Doxygen XML directory not found: {xml_dir}")

    api_reference_dir = args.api_reference_dir.resolve()
    if not api_reference_dir.is_dir():
        raise SystemExit(f"API reference directory not found: {api_reference_dir}")

    doc_targets = parse_doc_targets(api_reference_dir)
    exact_targets, api_compounds = parse_doxygen_xml(xml_dir)
    all_stale_files, partial_stale_files, missing_compounds = summarize(
        doc_targets, exact_targets, api_compounds, filters
    )
    compare_data = build_compare_lists(
        api_reference_dir,
        doc_targets,
        all_stale_files,
        partial_stale_files,
        missing_compounds,
    )

    if args.report == "json":
        print(
            json.dumps(
                {
                    "api_reference_root": str(api_reference_dir),
                    "doxygen_xml_dir": str(xml_dir),
                    "profile": args.profile,
                    "documented_targets": len(doc_targets),
                    "audited_exact_targets": len(exact_targets),
                    **compare_data,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    if args.report == "compare":
        print(f"API reference root: {api_reference_dir}")
        print(f"Doxygen XML: {xml_dir}")
        print(f"Profile: {args.profile}")
        print()
        print_compare(compare_data, args.show)
        return 1 if args.fail_on_findings and has_findings(compare_data) else 0

    print(f"API reference root: {api_reference_dir}")
    print(f"Doxygen XML: {xml_dir}")
    print(f"Documented targets: {len(doc_targets)}")
    print(f"Audited exact targets: {len(exact_targets)}")
    print(f"Profile: {args.profile}")
    print("Function directives are not audited strictly.")
    print()

    print_section("Files whose audited targets are all unresolved", [str(p) for p in all_stale_files], args.show)
    print()
    print_section(
        "Files whose audited targets are only partially unresolved",
        [str(p) for p in partial_stale_files],
        args.show,
    )
    print()
    print_section(
        "Public class/struct compounds with no wrapper page",
        [f"{compound.name} [{compound.file_path}]" for compound in missing_compounds],
        args.show,
    )

    return 1 if args.fail_on_findings and has_findings(compare_data) else 0


if __name__ == "__main__":
    sys.exit(main())
