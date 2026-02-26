!include "LogicLib.nsh"
!include "FileFunc.nsh"
!include "Registry.nsh"
!include "WordFunc.nsh"

; Insert required macros
!insertmacro WordFind

Name "Universal Launcher"
OutFile "UniversalLauncher.exe"
RequestExecutionLevel user
SilentInstall silent
Icon "${NSISDIR}\Contrib\Graphics\Icons\modern-install.ico"

Var IniFile
Var AppName
Var AppDir
Var ExeName
Var DataDir
Var RunAsAdmin
Var KillProcess
Var BackupFile
Var LaunchArgs
Var Wait
Var Count
Var Key
Var Val
Var EnvKey
Var EnvVal
Var TempVal
Var TempBackupFile
Var TmpVar
Var RegKey
Var RegValName
Var RegType
Var RegValue

Section "Main"
    StrCpy $IniFile "$EXEDIR\UniversalLauncher.ini"

    ; Read Main Settings
    ReadINIStr $AppName "$IniFile" "Main" "AppName"
    ReadINIStr $AppDir "$IniFile" "Main" "AppDir"
    ReadINIStr $ExeName "$IniFile" "Main" "ExeName"
    ReadINIStr $DataDir "$IniFile" "Main" "DataDir"
    ReadINIStr $RunAsAdmin "$IniFile" "Main" "RunAsAdmin"
    ReadINIStr $KillProcess "$IniFile" "Main" "KillProcess"

    ; Set Environment Variable for Expansion
    System::Call 'Kernel32::SetEnvironmentVariable(t "EXEDIR", t "$EXEDIR")'
    System::Call 'Kernel32::SetEnvironmentVariable(t "DataDir", t "$EXEDIR\$DataDir")'
    System::Call 'Kernel32::SetEnvironmentVariable(t "AppDir", t "$EXEDIR\$AppDir")'

    ; Admin Check
    ${If} $RunAsAdmin == "true"
        UserInfo::GetAccountType
        Pop $0
        ${If} $0 != "Admin"
            ExecShell "runas" "$EXEDIR\$EXEFILE"
            Quit
        ${EndIf}
    ${EndIf}

    ; Create Directories
    ReadINIStr $Count "$IniFile" "Directories" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Val "$IniFile" "Directories" "$0"
        ; Expand any env vars if present in directory path
        ExpandEnvStrings $Val "$Val"

        ; Check for absolute path (X:\ or \\)
        StrCpy $TmpVar $Val 2
        ${If} $TmpVar == "\\"
            ; UNC Path, assume absolute
        ${Else}
            StrCpy $TmpVar $Val 1 1
            ${If} $TmpVar == ":"
                ; Drive Letter, assume absolute
            ${Else}
                ; Relative Path, prepend EXEDIR
                StrCpy $Val "$EXEDIR\$Val"
            ${EndIf}
        ${EndIf}

        CreateDirectory "$Val"
    ${Next}

    ; Files Initialization (Copy if destination missing)
    ReadINIStr $Count "$IniFile" "FilesToCopy" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Val "$IniFile" "FilesToCopy" "$0"
        ${WordFind} "$Val" "|" "+1" $EnvKey  ; Source
        ${WordFind} "$Val" "|" "+2" $EnvVal  ; Dest

        ExpandEnvStrings $EnvKey "$EnvKey"
        ExpandEnvStrings $EnvVal "$EnvVal"

        ${If} ${FileExists} "$EnvVal"
             ; Dest exists, skip
        ${Else}
             CopyFiles "$EnvKey" "$EnvVal"
        ${EndIf}
    ${Next}

    ; Registry Backup
    ReadINIStr $BackupFile "$IniFile" "RegistryBackup" "BackupFile"
    ExpandEnvStrings $BackupFile "$BackupFile"

    ; Ensure BackupFile is absolute path
    StrCpy $TmpVar $BackupFile 2
    ${If} $TmpVar == "\\"
    ${Else}
        StrCpy $TmpVar $BackupFile 1 1
        ${If} $TmpVar == ":"
        ${Else}
            StrCpy $BackupFile "$EXEDIR\$BackupFile"
        ${EndIf}
    ${EndIf}

    ReadINIStr $Count "$IniFile" "RegistryBackup" "Count"
    ${For} $0 1 $Count
        StrCpy $TempBackupFile "$BackupFile_$0.reg"
        ${If} ${FileExists} "$TempBackupFile"
            ; Backup exists, skip export
        ${Else}
            ReadINIStr $Key "$IniFile" "RegistryBackup" "$0"
            nsExec::ExecToStack 'reg export "$Key" "$TempBackupFile" /y'
        ${EndIf}
    ${Next}

    ; Registry Cleanup (Delete Host Keys Before Launch)
    ReadINIStr $Count "$IniFile" "RegistryCleanup" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Key "$IniFile" "RegistryCleanup" "$0"
        nsExec::ExecToStack 'reg delete "$Key" /f'
    ${Next}

    ; Set Portable Registry Keys
    ReadINIStr $Count "$IniFile" "RegistryKeys" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Val "$IniFile" "RegistryKeys" "$0"
        ${WordFind} "$Val" "|" "+1" $RegKey
        ${WordFind} "$Val" "|" "+2" $RegValName
        ${WordFind} "$Val" "|" "+3" $RegType
        ${WordFind} "$Val" "|" "+4" $RegValue

        ExpandEnvStrings $RegValue "$RegValue"

        ${If} $RegType == "SZ"
            ${registry::Write} "$RegKey" "$RegValName" "$RegValue" "REG_SZ" $R0
        ${ElseIf} $RegType == "DWORD"
             ${registry::Write} "$RegKey" "$RegValName" "$RegValue" "REG_DWORD" $R0
        ${EndIf}
    ${Next}

    ; Process Kill
    ${If} $KillProcess != ""
        nsExec::ExecToStack 'taskkill /F /IM "$KillProcess"'
    ${EndIf}

    ; Set Environment Variables
    ReadINIStr $Count "$IniFile" "Environment" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Val "$IniFile" "Environment" "$0"
        ; Parse Key=Value
        ${WordFind} "$Val" "=" "+1" $EnvKey
        ${WordFind} "$Val" "=" "+2*" $EnvVal

        ExpandEnvStrings $EnvVal "$EnvVal"
        System::Call 'Kernel32::SetEnvironmentVariable(t "$EnvKey", t "$EnvVal")'
    ${Next}

    ; Launch
    ReadINIStr $LaunchArgs "$IniFile" "Launch" "Arguments"
    ExpandEnvStrings $LaunchArgs "$LaunchArgs"
    ReadINIStr $Wait "$IniFile" "Launch" "Wait"

    SetOutPath "$EXEDIR\$AppDir"

    ${If} $Wait == "true"
        ExecWait '"$EXEDIR\$AppDir\$ExeName" $LaunchArgs'
    ${Else}
        Exec '"$EXEDIR\$AppDir\$ExeName" $LaunchArgs'
    ${EndIf}

    ; Registry Cleanup (Delete Portable Keys Before Restore)
    ReadINIStr $Count "$IniFile" "RegistryCleanup" "Count"
    ${For} $0 1 $Count
        ReadINIStr $Key "$IniFile" "RegistryCleanup" "$0"
        nsExec::ExecToStack 'reg delete "$Key" /f'
    ${Next}

    ; Registry Restore (Import Host Keys)
    ReadINIStr $Count "$IniFile" "RegistryBackup" "Count"
    ${For} $0 1 $Count
        StrCpy $TempBackupFile "$BackupFile_$0.reg"
        ${If} ${FileExists} "$TempBackupFile"
            nsExec::ExecToStack 'reg import "$TempBackupFile"'
        ${EndIf}
    ${Next}

SectionEnd
