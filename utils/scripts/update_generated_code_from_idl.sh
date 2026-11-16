#!/usr/bin/env bash

files_to_exclude=(
    './code/DynamicTypesIDLExamples.idl'
    './code/Examples/C++/RpcClientServerFeed/src/types/calculator.idl'
)

files_not_needing_typeobject=(
    )

files_needing_case_sensitive=(
    )

files_needing_output_dir=(
    )

files_needing_no_typesupport=(
    )

red='\E[1;31m'
yellow='\E[1;33m'
textreset='\E[1;0m'

current_dir=$(git rev-parse --show-toplevel)

if [[ ! "$(pwd -P)" -ef "$current_dir" ]]; then
    echo -e "${red}This script must be executed in the repository root directory.${textreset}"
    exit -1
fi

if [[ -z "$(which fastddsgen)" ]]; then
    echo "Cannot find fastddsgen. Please, include it in PATH environment variable"
    exit -1
fi

readarray -d '' idl_files < <(find . -iname \*.idl -print0)

for del in ${files_to_exclude[@]}
do
    idl_files=("${idl_files[@]/$del}")
done

idl_files=(${idl_files[@]/$files_to_exclude})

ret_value=0

# CDR option check
if [[ $1 == "cdr-v1" ]]; then
    cdr_arg="-cdr v1"
elif [[ $1 == "cdr-v2" ]]; then
    cdr_arg="-cdr v2"
elif [[ $1 == "cdr-both" ]]; then
    cdr_arg="-cdr both"
else
    cdr_arg=""
fi

for idl_file in "${idl_files[@]}"; do
    idl_dir=$(dirname "$idl_file")
    file_from_gen=$(basename "$idl_file")

    echo -e "Processing ${yellow}$idl_file${textreset}"

    cd "${idl_dir}"

    # Detect if needs case sensitive.
    [[ ${files_needing_case_sensitive[*]} =~ $idl_file ]] && cs_arg='-cs' || cs_arg=''

    # Detect if needs output directories.
    not_processed=true
    for od_entry in ${files_needing_output_dir[@]}; do
        if [[ $od_entry = $idl_file\|* ]]; then
            not_processed=false;
            od_entry_split=(${od_entry//\|/ })
            for od_entry_split_element in ${od_entry_split[@]:1}; do
                od_arg="-d ${od_entry_split_element}"
                fastddsgen -replace $cdr_arg $cs_arg $od_arg "$file_from_gen"
            done
            break
        fi
    done

    if $not_processed ; then
        fastddsgen -replace $cdr_arg $cs_arg "$file_from_gen"
    fi

    if [[ $? != 0 ]]; then
        ret_value=-1
    fi

    cd -
done

exit $ret_value
