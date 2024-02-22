#!/bin/sh

if [ $# -eq 0 ]
    then
        echo "Usage: $0 <data_directory>"
        exit 1
    fi

data_dir=$1

# Check if provided directory exists
if [ ! -d "$data_dir" ]; then
    echo "Error: Directory '$data_dir' not found."
    exit 1
fi

echo "Data Directory: '$data_dir'"
docker run -it --rm -v "$data_dir":/data ca2lib