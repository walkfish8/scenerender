for /f %%i in (copy_release_dll.txt) do copy %%i .\build\bin\Release
for /f %%i in (copy_debug_dll.txt) do copy %%i .\build\bin\Debug
pause