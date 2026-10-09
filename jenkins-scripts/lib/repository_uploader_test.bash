#!/bin/bash

set -eu

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
uploader=${UPLOADER_UNDER_TEST:-"${script_dir}/repository_uploader.bash"}
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/repository-uploader-test.XXXXXX")
trap 'rm -rf "${test_dir}"' EXIT

mkdir -p "${test_dir}/bin" "${test_dir}/home" "${test_dir}/workspace/pkgs"
touch "${test_dir}/home/.s3cfg"

cat > "${test_dir}/bin/s3cmd" <<'EOF'
#!/bin/bash
printf 'uploaded\n' >> "${UPLOAD_LOG}"
EOF
chmod +x "${test_dir}/bin/s3cmd"

tarball="${test_dir}/workspace/pkgs/gz-sim-10.2.0.tar.bz2"
printf 'complete source tarball\n' > "${tarball}"
expected_sha256=$(sha256sum "${tarball}" | awk '{print $1}')

run_uploader()
{
    env \
      PATH="${test_dir}/bin:${PATH}" \
      HOME="${test_dir}/home" \
      WORKSPACE="${test_dir}/workspace" \
      UPLOAD_LOG="${test_dir}/uploads" \
      ENABLE_S3_UPLOAD=true \
      S3_UPLOAD_CANONICAL_PATH=false \
      UPLOAD_TO_REPO=only_s3_upload \
      S3_FILES_TO_UPLOAD=gz-sim-10.2.0.tar.bz2 \
      S3_UPLOAD_PATH=gz-sim/releases \
      SOURCE_TARBALL_URI=https://example.invalid/gz-sim-10.2.0.tar.bz2 \
      SOURCE_TARBALL_SHA256="${1}" \
      bash "${uploader}" > "${test_dir}/stdout" 2>&1
}

# A copied artifact that differs from the source job's checksum must fail
# before s3cmd is invoked.
if run_uploader "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"; then
    echo "Expected checksum mismatch to fail"
    exit 1
fi
if [[ -e "${test_dir}/uploads" ]]; then
    echo "Checksum mismatch reached s3cmd"
    exit 1
fi
grep -q "Source tarball checksum mismatch" "${test_dir}/stdout"

# The matching artifact must retain the existing successful upload behavior.
run_uploader "${expected_sha256}"
if [[ $(wc -l < "${test_dir}/uploads") -ne 1 ]]; then
    echo "Expected one upload for a matching checksum"
    exit 1
fi

echo "repository_uploader tests passed"
