#!/bin/bash

cd ../..
./waf configure
# Make sure there's a lorawan entry in index.rst
./waf docs
doxygen doc/doxygen.conf
mkdir api
cd api
git init .
rsync -rl ../doc/ ./
git remote add github-public git@github.com:signetlabdei/lorawan
ln -s html/index.html ./
cp ../src/lorawan/README.md ./
git add .
git commit -m "Update API"
git push -u origin gh-pages
cd ..
rm -rf api
