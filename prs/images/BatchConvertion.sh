#!/bin/sh
count=0
sourceType="pdf"
outputType="png"

echo "Converting all pdf's to png's at user selected dpi"

read -p "With what dpi should it be exported (e.g. 300)? " dpi

for fileSource in *.$sourceType
do
    if [ -f "$fileSource" ]; then    
        count=$((count+1))
        file=$(echo $fileSource | cut -d'.' -f1)
        echo $count". "$fileSource" -> "$file.$outputType
        inkscape $fileSource --export-$outputType=$file.$outputType --export-dpi=$dpi
    else
        echo "no file $fileSource found!"
    fi
done
echo "$count file(s) converted!"

