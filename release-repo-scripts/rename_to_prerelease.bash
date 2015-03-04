if [[ ${#} != 1 ]]; then
    echo "Usage: ${0} <packagename>"
    exit 1
fi

for i in *${1}*; do
    name=${i/.*}
    extension=${i#*.}
    new_file=${name}-prerelease.${extension}
    echo "$i -> ${new_file}"
    hg mv $i ${new_file}
done
