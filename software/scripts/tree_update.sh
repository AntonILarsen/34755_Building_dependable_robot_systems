#!/bin/bash

# Save current directory
CUR_DIR=$(pwd)

# Jump to root directory (where README.md is located)
cd "$(git rev-parse --show-toplevel)"

# Generate tree output with . removed and no listing of directory instances
TREE_OUTPUT=$(tree -d | tail -n +2 | grep -v "directories$")

# Fill in the updated directory structure.
# Write modified file to temporary file, and if successful then write it back to the original file.
awk -v tree="$TREE_OUTPUT" '
BEGIN {printIt=1}
/<!-- TREE START -->/ {
    print $0;    # Print contents before tree
    print "```"; # Start code block
    print tree;  # Print the tree
    print "```"; # End code block
    printIt=0    # Stop
}
/<!-- TREE END -->/ {printIt=1} # End fence found, now print the rest of the file
printIt {print $0} # Print line if printIt=1
' README.md > README.tmp && mv README.tmp README.md

cd "$CUR_DIR"
