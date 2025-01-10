#!/bin/bash

directory="."
patterns=("delete_all_files_except_patterns.sh" "vfm-includes-viper-env-model-simple.txt" "vfm-includes-viper-env-model.txt" "vfm-includes-viper_empty.txt" "vfm-includes-viper.txt" "evaluation.cpp" "rules.cpp" "commonFunctions.cpp" "vfm-options-viper.txt" "rule_based_planning_base_types.hpp" "definitions.hpp" "rule_based_planning_parameters.hpp" "dynamicObjects.hpp" "hmi.hpp" "map.hpp" "rule_based_planning_branching_conditions.hpp" "internalDefinitions.hpp" "envModel.hpp" "tree.hpp")

# Function to check if a file matches any of the patterns
matches_pattern() {
    local file="$1"
    for pattern in "${patterns[@]}"; do
        if [[ "$file" == *"$pattern"* ]]; then
            return 0
        fi
    done
    return 1
}

# Function to check if a directory is empty
is_empty_directory() {
    local dir="$1"
    if [ -z "$(ls -A "$dir")" ]; then
        return 0
    else
        return 1
    fi
}

# Recursive function to delete files
delete_files() {
    local dir="$1"
    shopt -s dotglob nullglob
    for file in "$dir"/*; do
        if [[ -d "$file" ]]; then
            delete_files "$file"
        elif [[ -f "$file" && ! -L "$file" ]]; then
            if ! matches_pattern "$file"; then
                rm "$file"
            fi
        fi
    done
}

# Recursive function to delete empty directories
delete_empty_directories() {
    local dir="$1"
    for file in "$dir"/*; do
        if [ -d "$file" ]; then
            delete_empty_directories "$file"
            if is_empty_directory "$file"; then
                rmdir "$file"
            fi
        fi
    done
}

# Start deleting files from the directory
delete_files "$directory"
delete_empty_directories "$directory"
