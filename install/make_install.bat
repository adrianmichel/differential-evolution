@echo off
pushd %~dp0

call ..\doc_src\make_doc.bat
set zip="\program files\7-zip\7z.exe"

del *.zip
call %zip% a -tzip de.zip -ir@include.txt  -xr@exclude.txt 

call %zip% a -tzip de_amichel.com.zip de.zip ..\doc\

popd

