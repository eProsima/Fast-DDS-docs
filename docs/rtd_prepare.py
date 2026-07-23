# -*- coding: utf-8 -*-
"""
Generate the auto-generated API reference sources for the Fast DDS docs.

This performs the expensive preparation steps of the documentation build:

  1. Shallow-clone Fast DDS (C++ headers) and Fast DDS Python bindings.
  2. Run doxygen over the C++ headers to produce the XML consumed by breathe.
  3. Run SWIG to generate the Python bindings source consumed by autodoc.

Nothing is *compiled* here: doxygen only parses headers, and SWIG only
generates source. The compiled extension `_fastdds_python` is mocked by
autodoc (see conf.py).

It is meant to be run by Read the Docs as a `pre_build` job (see
readthedocs.yaml) so these steps run ONCE, and are logged with their own
timing as a dedicated build step, instead of being executed inside conf.py
on every single sphinx-build invocation (html, and then latex for the PDF).
"""
import os
import shutil
import subprocess
import sys
import time

import git


def log(message):
    print(message, flush=True)


def run(command):
    """Run a shell command, streaming its output; abort the build on error."""
    log('$ {}'.format(command))
    start = time.monotonic()
    ret = subprocess.call(command, shell=True)
    log('-> exit {} in {:.1f}s'.format(ret, time.monotonic() - start))
    if ret != 0:
        sys.exit(ret)


def shallow_clone(url, path, requested_branch, default_branch):
    """Shallow-clone a single branch (or tag); fall back to default if missing."""
    branch = requested_branch or default_branch
    for candidate in (branch, default_branch):
        try:
            log('Cloning "{}" branch "{}" into "{}"'.format(url, candidate, path))
            start = time.monotonic()
            git.Repo.clone_from(
                url,
                path,
                multi_options=['--depth=1', '--single-branch'],
                branch=candidate,
            )
            log('-> cloned in {:.1f}s'.format(time.monotonic() - start))
            return
        except git.GitCommandError as e:
            log('Branch "{}" not available ({}).'.format(candidate, e))
    log('Failed to clone {}'.format(url))
    sys.exit(1)


def configure_doxyfile(doxyfile_in, doxyfile_out, replacements):
    """Configure the Doxyfile in the CMake style (replace @VAR@ placeholders)."""
    log('Configuring Doxyfile {}'.format(doxyfile_out))
    with open(doxyfile_in, 'r') as f:
        data = f.read()
    for key, value in replacements.items():
        data = data.replace(key, value)
    os.makedirs(os.path.dirname(doxyfile_out), exist_ok=True)
    with open(doxyfile_out, 'w') as f:
        f.write(data)


def main():
    total_start = time.monotonic()

    script_path = os.path.abspath(os.path.dirname(__file__))
    project_source_dir = os.path.abspath('{}/../code'.format(script_path))
    project_binary_dir = os.path.abspath('{}/../build'.format(script_path))
    output_dir = os.path.abspath('{}/doxygen'.format(project_binary_dir))
    doxygen_html = os.path.abspath('{}/html/doxygen'.format(project_binary_dir))
    doxyfile_in = os.path.abspath(
        '{}/doxygen-config.in'.format(project_source_dir))
    doxyfile_out = os.path.abspath(
        '{}/doxygen-config'.format(project_binary_dir))
    input_dir = os.path.abspath(
        '{}/fastdds/include/fastdds'.format(project_binary_dir))
    fastdds_repo_name = os.path.abspath(
        '{}/fastdds'.format(project_binary_dir))
    fastdds_python_repo_name = os.path.abspath(
        '{}/fastdds_python'.format(project_binary_dir))

    log('==> Preparing Fast DDS API reference sources')

    # Clean previous clones
    for repo in (fastdds_repo_name, fastdds_python_repo_name):
        if os.path.isdir(repo):
            log('Removing existing repository {}'.format(repo))
            shutil.rmtree(repo)
    os.makedirs(os.path.dirname(fastdds_repo_name), exist_ok=True)
    os.makedirs(os.path.dirname(fastdds_python_repo_name), exist_ok=True)

    # 1. Clone (shallow, single branch)
    log('==> [1/3] Cloning repositories')
    shallow_clone(
        'https://github.com/eProsima/Fast-DDS.git',
        fastdds_repo_name,
        os.environ.get('FASTDDS_BRANCH', None),
        '2.6.x',
    )
    shallow_clone(
        'https://github.com/eProsima/Fast-DDS-python.git',
        fastdds_python_repo_name,
        os.environ.get('FASTDDS_PYTHON_BRANCH', None),
        '1.1.x',
    )

    # 2. Doxygen: C++ headers -> XML (consumed by breathe)
    log('==> [2/3] Generating doxygen XML')
    os.makedirs(os.path.dirname(output_dir), exist_ok=True)
    os.makedirs(os.path.dirname(doxygen_html), exist_ok=True)
    configure_doxyfile(
        doxyfile_in,
        doxyfile_out,
        {
            '@DOXYGEN_INPUT_DIR@': input_dir,
            '@DOXYGEN_OUTPUT_DIR@': output_dir,
            '@PROJECT_BINARY_DIR@': project_binary_dir,
            '@PROJECT_SOURCE_DIR@': project_source_dir,
        },
    )
    run('doxygen {}'.format(doxyfile_out))

    # 3. SWIG: generate Python bindings source (consumed by autodoc)
    log('==> [3/3] Generating SWIG Python bindings source')
    run(
        'swig -python -doxygen -I{repo}/include '
        '-outdir {py}/src/swig -c++ -interface _fastdds_python '
        '-o {py}/src/swig/fastddsPYTHON_wrap.cxx '
        '{py}/src/swig/fastdds.i'.format(
            repo=fastdds_repo_name,
            py=fastdds_python_repo_name,
        )
    )

    log('==> Done in {:.1f}s'.format(time.monotonic() - total_start))


if __name__ == '__main__':
    main()
