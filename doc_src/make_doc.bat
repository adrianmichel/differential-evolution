@echo off
pushd %~dp0

rd /Q /S ..\doc
"\program files\doxygen\bin\doxygen.exe" "de.cfg"

popd

