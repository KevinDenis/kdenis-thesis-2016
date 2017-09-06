#!/bin/sh
count=0
sourceType="pdf"
outputType="png"
dpi=300

echo "Converting all pdf's to png's @ 300 dpi"

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

