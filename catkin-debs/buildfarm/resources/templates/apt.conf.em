@{import os}
Dir::Etc @os.path.join(rootdir, 'etc/apt');
Dir::State @os.path.join(rootdir, 'var/lib/apt');
