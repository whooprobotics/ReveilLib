#!/bin/bash

mkdir copro_package 
find include -name "*.hh" | grep -E '(common|copro)' | xargs -I {} cp --parents {} copro_package
cp bin/reveillib_copro.so copro_package
zip -r copro_package.zip copro_package
