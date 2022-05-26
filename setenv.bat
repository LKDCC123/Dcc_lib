@color 17
@set CURRENT_DISC=%~d0
@set CURRENT_PATH=%cd%
@set VS_DISC=C:
@set VS_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build
@%VS_DISC%
@cd %VS_PATH%
@call vcvars64.bat
@%CURRENT_DISC%
@cd %CURRENT_PATH%
@color 16
