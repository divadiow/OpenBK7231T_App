param(
    [string]$Port = 'COM45'
)

$ErrorActionPreference = 'Stop'

$sp = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'One'
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout = 20
$sp.WriteTimeout = 1000

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$base = Join-Path $PSScriptRoot "TXW817_DIAG_V04_$stamp"
$logPath = "$base.log"
$rxPath = "$base.rx.bin"
$txPath = "$base.tx.bin"

$rxAll = New-Object 'System.Collections.Generic.List[byte]'
$txAll = New-Object 'System.Collections.Generic.List[byte]'

function Log {
    param([string]$Text)

    $line = ('[{0:HH:mm:ss.fff}] {1}' -f (Get-Date), $Text)
    Write-Host $line
    Add-Content -LiteralPath $logPath -Value $line -Encoding UTF8
}

function Add-TxBytes {
    param([byte[]]$Bytes)
    foreach ($b in $Bytes) {
        [void]$txAll.Add($b)
    }
}

function Add-RxBytes {
    param(
        [byte[]]$Bytes,
        [int]$Count
    )

    for ($i = 0; $i -lt $Count; $i++) {
        [void]$rxAll.Add($Bytes[$i])
    }
}

function Show-Rx {
    param(
        [byte[]]$Bytes,
        [int]$Count
    )

    Add-RxBytes -Bytes $Bytes -Count $Count

    $hex = New-Object 'System.Collections.Generic.List[string]'
    $ascii = New-Object System.Text.StringBuilder

    for ($i = 0; $i -lt $Count; $i++) {
        $b = $Bytes[$i]
        [void]$hex.Add(('{0:X2}' -f $b))

        if ($b -ge 32 -and $b -le 126) {
            [void]$ascii.Append([char]$b)
        }
        else {
            [void]$ascii.Append('.')
        }
    }

    Log ('RX: ' + ($hex -join ' ') + '  |' + $ascii.ToString() + '|')
}

function Read-Some {
    param([int]$MaxBytes = 256)

    $available = $sp.BytesToRead
    if ($available -le 0) {
        return $null
    }

    $take = [Math]::Min($available, $MaxBytes)
    if ($take -lt 1) {
        return $null
    }

    $buffer = New-Object byte[] $take
    $got = 0

    try {
        $got = $sp.Read($buffer, 0, $take)
    }
    catch [System.TimeoutException] {
        $got = 0
    }

    if ($got -le 0) {
        return $null
    }

    Show-Rx -Bytes $buffer -Count $got

    $result = New-Object byte[] $got
    [Array]::Copy($buffer, 0, $result, 0, $got)
    return $result
}

function Wait-Text {
    param(
        [string]$Needle,
        [int]$TimeoutMs
    )

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $text = New-Object System.Text.StringBuilder

    while ($sw.ElapsedMilliseconds -lt $TimeoutMs) {
        $chunk = Read-Some
        if ($null -ne $chunk) {
            foreach ($b in $chunk) {
                [void]$text.Append([char]$b)
            }

            if ($text.ToString().Contains($Needle)) {
                return $true
            }
        }
        Start-Sleep -Milliseconds 2
    }

    return $false
}

function Write-SerialBytes {
    param(
        [byte[]]$Bytes,
        [string]$Label
    )

    Log "TX: $Label"
    Add-TxBytes -Bytes $Bytes
    $sp.Write($Bytes, 0, $Bytes.Length)
}

function Write-SerialLine {
    param([string]$Text)

    $bytes = [System.Text.Encoding]::ASCII.GetBytes($Text + "`r`n")
    Write-SerialBytes -Bytes $bytes -Label $Text
}

function Wait-RomAck {
    param([int]$TimeoutMs)

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $rolling = New-Object 'System.Collections.Generic.List[byte]'

    while ($sw.ElapsedMilliseconds -lt $TimeoutMs) {
        $chunk = Read-Some
        if ($null -ne $chunk) {
            foreach ($b in $chunk) {
                [void]$rolling.Add($b)
                if ($rolling.Count -gt 3) {
                    $rolling.RemoveAt(0)
                }

                if (
                    $rolling.Count -eq 3 -and
                    $rolling[0] -eq 0x18 -and
                    $rolling[1] -eq 0x18 -and
                    $rolling[2] -eq 0x18
                ) {
                    return $true
                }
            }
        }
        Start-Sleep -Milliseconds 2
    }

    return $false
}

try {
    $sp.Open()
    $sp.DiscardInBuffer()
    $sp.DiscardOutBuffer()

    Log "Opened $Port at 115200 8N1; DTR/RTS disabled."
    Log 'FIXED WIRING FOR BOTH APPLICATION AND MASK ROM:'
    Log '  USB-TTL RX <- TXW817 PA9  [target TX]'
    Log '  USB-TTL TX -> TXW817 PA8  [target RX]'
    Log '  GND        <-> GND'
    Log 'Waiting for TXWDIAG READY v=0.4...'

    if (-not (Wait-Text -Needle 'TXWDIAG READY v=0.4' -TimeoutMs 3000)) {
        throw 'READY v0.4 not seen. Reset/power-cycle target while script is already running.'
    }

    Write-SerialLine -Text 'AT+DIAG=PING'
    if (-not (Wait-Text -Needle 'TXWDIAG PONG v=0.4' -TimeoutMs 500)) {
        throw 'PING failed.'
    }

    Write-SerialLine -Text 'AT+DIAG=SNAPSHOT'
    if (-not (Wait-Text -Needle 'TXWDIAG SNAP' -TimeoutMs 500)) {
        throw 'SNAPSHOT failed.'
    }

    $nonce = Get-Random -Minimum 1 -Maximum 2147483647
    $nonceHex = '{0:X8}' -f $nonce

    Write-SerialLine -Text "AT+DIAG=ARM,$nonceHex"
    if (-not (Wait-Text -Needle "TXWDIAG ARMED 0x$nonceHex" -TimeoutMs 600)) {
        throw 'ARM failed.'
    }

    Log 'Sending EXEC immediately; no human delay.'
    Write-SerialLine -Text "AT+DIAG=EXEC,$nonceHex"
    if (-not (Wait-Text -Needle 'TXWDIAG ROMGO' -TimeoutMs 600)) {
        throw 'ROMGO failed.'
    }

    Log 'ROMGO received. Waiting 500 ms for watchdog reset and mask-ROM UART initialization.'
    Start-Sleep -Milliseconds 500

    # The mask ROM is now using the SAME physical wiring as the application:
    # PA8 RX and PA9 TX. Send the proven textual entry token once only.
    $bootl = [System.Text.Encoding]::ASCII.GetBytes('AT+BOOTL')
    Write-SerialBytes -Bytes $bootl -Label 'exact AT+BOOTL, 8 bytes, no CR/LF, once'

    if (-not (Wait-RomAck -TimeoutMs 1200)) {
        throw 'No 18 18 18 mask-ROM acknowledgement after the single AT+BOOTL.'
    }

    Log 'PASS: mask ROM replied 18 18 18 with no signal-wire swap.'
    Log 'STOPPING HERE: no binary ROM command was sent.'
    exit 0
}
finally {
    try {
        if ($sp.IsOpen) {
            $sp.Close()
        }
    }
    catch {
    }

    try {
        [System.IO.File]::WriteAllBytes($rxPath, $rxAll.ToArray())
        [System.IO.File]::WriteAllBytes($txPath, $txAll.ToArray())
        Log "Saved RX: $rxPath"
        Log "Saved TX: $txPath"
        Log "Saved log: $logPath"
    }
    catch {
    }
}
