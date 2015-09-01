!include "MUI2.nsh"
!include "LogicLib.nsh"

; Define section
!define ProgramName        "Skeleton Pose Estimation Library"
!define ProgramVersion     "v1.0.0.0"
!define CompanyName        "Skeleton Pose Estimation Library"
!define FullInstType       "Full"
!define LibrarySection     "SPEL library"
!define SolverTestsSection "Solver Tests"
!define TestDataSection    "Test Data (http://humaneva.is.tue.mpg.de/)"

SetCompress force
SetDateSave on
SetDatablockOptimize on
CRCCheck on
SetCompressor /SOLID lzma

Name "${ProgramName}"
Caption "${ProgramName}"
OutFile "${ProgramName} ${ProgramVersion} x64.exe"

InstallDir "$PROGRAMFILES64\${CompanyName}\${ProgramName}"
InstallDirRegKey HKLM "Software\${CompanyName}\${ProgramName}" "Installation Directory"

!define MUI_ICON "${NSISDIR}\Contrib\Graphics\Icons\orange-install-nsis.ico"
!define MUI_UNICON "${NSISDIR}\Contrib\Graphics\Icons\orange-uninstall-nsis.ico"

!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Header\orange-nsis.bmp"
!define MUI_HEADERIMAGE_UNBITMAP "${NSISDIR}\Contrib\Graphics\Header\orange-uninstall-nsis.bmp"
!define MUI_WELCOMEFINISHPAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Wizard\orange-nsis.bmp"
!define MUI_UNWELCOMEFINISHPAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Wizard\orange-uninstall-nsis.bmp"

!define MUI_ABORTWARNING

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE "../LICENSE"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_WELCOME
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

!ifndef NOINSTTYPES ; only if not defined
  InstType "${FullInstType}"
!endif

Section "${LibrarySection}"

SectionIn RO

SetOutPath $INSTDIR\include
File /r "spel\*.hpp"

SetOutPath $INSTDIR\lib
File /r "..\build\spel\Release\*.lib"

WriteRegStr HKLM "Software\${CompanyName}\${ProgramName}" "Installation Directory" "$INSTDIR"
WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ProgramName}" "DisplayName" "${ProgramName}"
WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ProgramName}" "UninstallString" '"$INSTDIR\uninstall.exe"'
WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ProgramName}" "NoModify" 1
WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ProgramName}" "NoRepair" 1
WriteUninstaller "uninstall.exe"

CreateDirectory "$SMPROGRAMS\${CompanyName}\${ProgramName}"
CreateShortCut "$SMPROGRAMS\${CompanyName}\${ProgramName}\Uninstall.lnk" "$INSTDIR\uninstall.exe" "" "$INSTDIR\uninstall.exe" 0

SectionEnd

Section "${SolverTestsSection}"

SectionIn 1

SetOutPath $INSTDIR\solverTests
File /r "..\build\utils\solverTests\Release\*"

SectionEnd

Section "${TestDataSection}"

SectionIn 1

SetOutPath $TEMP

NSISdl::download https://github.com/spelteam/spel/releases/download/1.0.0.0/HE_Box_1_S2_C1.zip $TEMP\HE_Box_1_S2_C1.zip
Pop $0
${If} $0 == "success"
	SetOutPath $INSTDIR\TestData
	ZipDLL::extractall $TEMP\HE_Box_1_S2_C1.zip $INSTDIR\TestData
${Else}
	MessageBox mb_iconstop "Error: $0"
${EndIf}

SectionEnd

Section "Uninstall"
  
DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ProgramName}"
DeleteRegKey HKLM "SOFTWARE\${CompanyName}\${ProgramName}"

Delete "$SMPROGRAMS\${CompanyName}\${ProgramName}\*"
Delete "$DESKTOP\${ProgramName}.lnk"

RMDir "$SMPROGRAMS\${CompanyName}\${ProgramName}"
RMDir /r /REBOOTOK "$INSTDIR"

SectionEnd
