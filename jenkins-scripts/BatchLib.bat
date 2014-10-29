REM ************** BEGIN OF BATCHLIB **************
REM * A "real" batch library system, that allowes to import other batch files into a batch application
REM *
REM * written 2010 by jeb
REM *******************
@echo off
setlocal EnableDelayedExpansion
set "param1=%~1"

REM *** Decide if the Library is started for initalization or only for the first prepare
if "!param1!"=="/INTERN_START" goto :bl_InternalStart

if /i "!param1!" EQU "/S" (
   REM ** Secure start, retrieve the parameter in a secure way, it accepts nearly every parameter, but it needs a temporary file
   REM ** TODO: Implement the REM redirect, Problems: access more than %1 ..%9 (shift, detect the end), double expansion of %%
   REM ** Buy it now for only 399Euro :-)
   echo NOT IMPLEMENTED
   EXIT
) ELSE (
   REM ** Basic start, retrieve the parameter in a simple way, it fails with complex parameters like "&"&"&
   REM ** TODO: Accept more than %9 parameters
   set BL.Start=#":< nul ( setlocal EnableDelayedExpansion & set "bl.cmdcmdline=^^!cmdcmdline^^!" & call set "bl.param[0]=%%~f0" & FOR /L %%n IN (1,1,9) DO ( call set "bl.param[%%n]=%%~1" & shift /1 ) )#"^
#"   & "%~f0" /INTERN_START "^^!bl.param[0]^^!"#"
)
(
  ENDLOCAL
  set "BL.Start=%BL.Start:#"=%"
  goto :eof
)

:bl_InternalStart
rem echo Library init from "%param[0]%"

rem Create Temporary Batchfile with library functions

rem Create first line with call to library_init and jump to the application, suppress a line ending for consistent line numbers
> "%~n2.tmp"  <nul set /p ".=call :BL.Init & goto :%%%%Bl.Start%%%% & rem "
rem Append the original batch file
>> "%~n2.tmp" type "%~f2"

REM Append a "good" file stop mark
(
echo(
echo ^)
echo goto :eof
) >> "%~n2.tmp"

rem Append this library
>> "%~n2.tmp" type "%~f0"

del "%~n2.BLibTmp.bat" 2> nul > nul
ren "%~n2.tmp" "%~n2.BLibTmp.bat" > nul

rem Asynchron start, creates a new cmd instance in the same window
rem Required, even if the application exit or crash the library can do the rest
rem (set dummy=) | ( call "%~n1.BLibTmp.bat" "%~2" )

REM Block this, so it is cached and not depends on a file
(
   call "%~n2.BLibTmp.bat"
   rem start "" /wait /b "%~n1.BLibTmp.bat" "%~2"
   REM echo End of application, remove temporary file^(s^)
   set "err=%errorlevel%"
   del "%~n2.BLibTmp.bat" > nul
   exit /b !err!
)
:: End of function

:BL.Init
rem At this moment there is nothing to do
call :BL.DragAndDrop.Parser
goto :eof
:: End of function

:: Imports/append the file to the current batch file
:BL.Import [filename.bat]
(
   setlocal EnableDelayedExpansion
   type "%~1" >> "%~dpf0"
   set "func=%~n1.Init"
   set "func=!func:_=.!"
)
(
   endlocal
   call :%func%
   goto :eof
)
:: End of function

:: Builds a drag & drop filelist, this list should be used, because files like "Drag&drop.bat" passed in the wrong way by windows
:: Therefore a Drag&Drop Batchfile should always end with an EXIT to suppress the execution of further commands after ampersands
rem Take the cmd-line, remove all until the first parameter
:BL.DragAndDrop.Parser
(
   set "$$$.params=!bl.cmdcmdline:~0,-1!"
   set "$$$.params=!$$$.params:*" =!"
   set bl.dragDrop.count=0

   rem Split the parameters on spaces but respect the quotes
   for %%G IN (!$$$.params!) do (
     set /a bl.dragDrop.count+=1
     set "bl.dragDrop[!bl.dragDrop.count!]=%%~G"
   )
   set "$$$.params="
   goto :eof
)
:: End of function
