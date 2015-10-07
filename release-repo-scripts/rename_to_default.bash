for i in *-prerelease*; do
    name=${i/.*}
    extension=${i#*.}
    new_file=${name//-prerelease}.${extension}
    echo "$i -> ${new_file}"
    hg mv $i ${new_file}
done
