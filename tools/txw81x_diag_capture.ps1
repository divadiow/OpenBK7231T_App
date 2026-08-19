param(
    [string]$Port = 'COM47'
)

$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'One'
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.ReadTimeout = 20
$sp.WriteTimeout = 1000
$sp.NewLine = "`r`n"

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$log = Join-Path $PSScriptRoot "TXW817_DIAG_${stamp}.log"
$raw = New-Object System.Collections.Generic.List[byte]

function Log([string]$s) {
    $line = ('[{0:HH:mm:ss.fff}] {1}' -f (Get-Date), $s)
    Write-Host $line
    Add-Content -LiteralPath $log -Value $line
}

function Read-For([int]$Milliseconds) {
    $until = [Environment]::TickCount64 + $Milliseconds
    $text = New-Object System.Text.StringBuilder
    while ([Environment]::TickCount64 -lt $until) {
        try {
            while ($sp.BytesToRead -gt 0) {
                $b = $sp.ReadByte()
                if ($b -ge 0) {
                    $raw.Add([byte]$b)
                    [void]$text.Append([char]$b)
                    Write-Host -NoNewline ([char]$b)
                }
            }
        } catch [TimeoutException] {}
        Start-Sleep -Milliseconds 5
    }
    return $text.ToString()
}

function Send-Line([string]$s) {
    Log "TX: $s"
    $sp.Write($s + "`r`n")
}

try {
    $sp.Open()
    Log "Opened $Port at 115200 8N1. DTR/RTS disabled."
    Log 'Listening 1500 ms for TXWDIAG READY...'
    [void](Read-For 1500)

    Send-Line 'AT+DIAG=PING'
    $r = Read-For 1200
    if ($r -notmatch 'TXWDIAG PONG v=0\.3') {
        Log 'FAIL: no v0.3 PONG. Stop here.'
        exit 2
    }
    Log 'PASS: PONG received.'

    Send-Line 'AT+DIAG=SNAPSHOT'
    [void](Read-For 1200)

    $nonce = Get-Random -Minimum 1 -Maximum 2147483647
    $nonceHex = '{0:X8}' -f $nonce
    Send-Line "AT+DIAG=ARM,$nonceHex"
    $r = Read-For 1500
    if ($r -notmatch "TXWDIAG ARMED 0x$nonceHex") {
        Log 'FAIL: ARM acknowledgement missing. Stop here.'
        exit 3
    }

    Write-Host
    Read-Host 'Snapshot captured. Press ENTER to execute system_goto_boot, or Ctrl+C to abort'
    Send-Line "AT+DIAG=EXEC,$nonceHex"
    $r = Read-For 1500
    if ($r -notmatch 'TXWDIAG ROMGO') {
        Log 'FAIL: ROMGO acknowledgement missing. Stop here.'
        exit 4
    }

    Start-Sleep -Milliseconds 120
    $bootl = [Text.Encoding]::ASCII.GetBytes('AT+BOOTL')
    Log 'TX: exact AT+BOOTL, no CR/LF, once'
    $sp.Write($bootl,0,$bootl.Length)

    $deadline = [Environment]::TickCount64 + 2500
    $last = New-Object System.Collections.Generic.List[byte]
    $success = $false
    while ([Environment]::TickCount64 -lt $deadline) {
        try {
            while ($sp.BytesToRead -gt 0) {
                $b = [byte]$sp.ReadByte()
                $raw.Add($b)
                $last.Add($b)
                if ($last.Count -gt 3) { $last.RemoveAt(0) }
                Write-Host -NoNewline ('{0:X2} ' -f $b)
                if ($last.Count -eq 3 -and $last[0] -eq 0x18 -and $last[1] -eq 0x18 -and $last[2] -eq 0x18) {
                    $success = $true
                    break
                }
            }
        } catch [TimeoutException] {}
        if ($success) { break }
        Start-Sleep -Milliseconds 5
    }
    Write-Host

    if ($success) {
        Log 'PASS: ROM replied 18 18 18. Application -> mask ROM transition proven.'
        exit 0
    }
    Log 'FAIL: no 18 18 18 after the single AT+BOOTL.'
    exit 5
}
finally {
    try { if ($sp.IsOpen) { $sp.Close() } } catch {}
    if ($raw.Count -gt 0) {
        [IO.File]::WriteAllBytes((Join-Path $PSScriptRoot "TXW817_DIAG_${stamp}.raw.bin"), $raw.ToArray())
    }
    Log "Log: $log"
}
