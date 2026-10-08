@echo off
:: Ask `gzdev filter-tests` whether a pull request needs its tests.
::
:: Usage: filter_tests.bat <repository directory relative to WORKSPACE> <none rc>
::
:: Exits <none rc> when the change needs no tests. Exits 0 in every other
:: case, including any problem, so the job runs its normal build.
::
:: Environment: WORKSPACE, ghprbTargetBranch, ghprbActualCommit and
:: ghprbSourceBranch (set by Jenkins and ghprb). GZDEV_URL (optional) is the
:: repository to clone gzdev from.
setlocal

set REPO_DIR=%WORKSPACE%\%~1
set NONE_RC=%~2
set GZDEV_DIR=%WORKSPACE%\gzdev-filter-tests
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
  echo filter_tests.bat: cannot get gzdev: running the full build
  exit /b 0
)

:: Reports left by an earlier build must not be published next to the stub
if exist "%WORKSPACE%\build\test_results" rmdir /s /q "%WORKSPACE%\build\test_results"
python "%GZDEV_DIR%\gzdev.py" filter-tests --build-platform=jenkins --repo-path="%REPO_DIR%" --return-if-none=%NONE_RC% --junit-output="%WORKSPACE%\build\test_results\filter_tests.xml"
set RC=%ERRORLEVEL%

if "%RC%" == "%NONE_RC%" (
  echo filter_tests.bat: no tests needed for this change
  exit /b %NONE_RC%
)
if not "%RC%" == "0" echo filter_tests.bat: gzdev filter-tests could not decide, rc %RC%: running the full build
exit /b 0
