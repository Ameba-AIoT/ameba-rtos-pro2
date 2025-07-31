rem copy files to release folder
rem Cloud Tseng
@echo off
echo ... copy release files ...

SET FOLDER_NAME=.\
SET MYPATHCOPY=%cd%
:search
for /f "delims=\ tokens=1,*" %%p in ("%MYPATHCOPY%") do (
	@echo %%~p
	SET FOLDER_NAME=%%~p
	SET MYPATHCOPY=%%~q\
)
if "%MYPATHCOPY%"=="\" goto done
echo %FOLDER_NAME%
goto search
:done

for /f "tokens=1,2,3* delims= " %%i in (auto_release.txt) do ( 
	rem copy files
	if %%i == f (
		echo f | xcopy "%%j" "release-%FOLDER_NAME%\%%j" /y
	)
	if %%i == fr (
		echo f | xcopy "%%j" "release-%FOLDER_NAME%\%%k" /y							
	)
	rem copy dir
	if %%i == d (
		if not exist release-%FOLDER_NAME%\%%j mkdir release-%FOLDER_NAME%\%%j
		xcopy "%%j\*.*" "release-%FOLDER_NAME%\%%j\" /e/y
	)
)

pause
