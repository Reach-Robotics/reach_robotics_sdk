#!/bin/bash

# Command file for Sphinx documentation

if [ -z "$SPHINXBUILD" ]; then
	SPHINXBUILD="sphinx-build"
fi

SOURCEDIR="source"
BUILDDIR="build"

if [ -z "$1" ]; then
	echo "Usage: $0 [command]"
	echo "Available commands:"
	echo "  github - Generate HTML documentation and copy to ../docs"
	exit 1
fi

if [ "$1" == "github" ]; then
	$SPHINXBUILD -M html $SOURCEDIR $BUILDDIR $SPHINXOPTS
	rm -r ../docs
	cp -r $BUILDDIR/html ../docs
	touch ../docs/.nojekyll
	echo "Generated files copied to ../docs"
	exit 0
fi

$SPHINXBUILD >/dev/null 2>&1
if [ $? -eq 9009 ]; then
	echo ""
	echo "The 'sphinx-build' command was not found. Make sure you have Sphinx"
	echo "installed, then set the SPHINXBUILD environment variable to point"
	echo "to the full path of the 'sphinx-build' executable. Alternatively, you"
	echo "may add the Sphinx directory to PATH."
	echo ""
	echo "If you don't have Sphinx installed, grab it from"
	echo "http://sphinx-doc.org/"
	exit 1
fi

$SPHINXBUILD -M $1 $SOURCEDIR $BUILDDIR $SPHINXOPTS $O

exit 0