@echo off
:: Ask `gzdev filter-build-and-test` whether a pull request needs a build and
:: tests.
::
:: Usage: filter_build_and_test.bat <repository directory relative to WORKSPACE> <none rc>
::
:: Exits <none rc> when the change needs no build or tests. Exits 0 in every
:: other case, including any problem, so the job runs its normal build.
::
:: Environment: WORKSPACE, ghprbTargetBranch, ghprbActualCommit and
:: ghprbSourceBranch (set by Jenkins and ghprb). GZDEV_URL (optional) is the
:: repository to clone gzdev from.
setlocal

set REPO_DIR=%WORKSPACE%\%~1
set NONE_RC=%~2
set GZDEV_DIR=%WORKSPACE%\gzdev-filter-build-and-test
if "%GZDEV_URL%" == "" set GZDEV_URL=https://github.com/gazebo-tooling/gzdev
set GZDEV_BRANCH=master

if exist "%GZDEV_DIR%" rmdir /s /q "%GZDEV_DIR%"
if "%ghprbSourceBranch%" == "" goto :clone
python "%~dp0..\tools\detect_ci_matching_branch.py" "%ghprbSourceBranch%"
if errorlevel 1 goto :clone
git ls-remote --exit-code --heads %GZDEV_URL% "%ghprbSourceBranch%"
if errorlevel 1 goto :clone
set GZDEV_BRANCH=%ghprbSourceBranch%

:clone
git clone --quiet --depth 1 --branch "%GZDEV_BRANCH%" %GZDEV_URL% "%GZDEV_DIR%"
if errorlevel 1 (
  echo filter_build_and_test.bat: cannot get gzdev: running the full build
  exit /b 0
)

:: Reports left by an earlier build must not be published next to the stub
if exist "%WORKSPACE%\build\test_results" rmdir /s /q "%WORKSPACE%\build\test_results"
python "%GZDEV_DIR%\gzdev.py" filter-build-and-test --build-platform=jenkins --repo-path="%REPO_DIR%" --return-if-none=%NONE_RC% --junit-output="%WORKSPACE%\build\test_results\filter_build_and_test.xml"
set RC=%ERRORLEVEL%

if "%RC%" == "%NONE_RC%" (
  echo filter_build_and_test.bat: no build or tests needed for this change
  exit /b %NONE_RC%
)
if not "%RC%" == "0" echo filter_build_and_test.bat: gzdev filter-build-and-test could not decide, rc %RC%: running the full build
exit /b 0
