# Render all Mermaid diagrams to PNG and SVG
# UAV Flight Controller - MBSE Diagram Rendering Script

$sourceRoot = "c:\Users\ratan\Desktop\End-to-End-RL-agent-UAV-flight-controller\WBS\diagrams\source"
$renderedRoot = "c:\Users\ratan\Desktop\End-to-End-RL-agent-UAV-flight-controller\WBS\diagrams\rendered"

# Create rendered directory structure
$categories = @("architecture", "process", "dataflow", "timeline", "components", "behavioral", "requirements", "deployment")

foreach ($category in $categories) {
    $dir = Join-Path $renderedRoot $category
    if (-not (Test-Path $dir)) {
        New-Item -ItemType Directory -Path $dir -Force | Out-Null
        Write-Host "Created: $dir" -ForegroundColor Green
    }
}

# Find all .mmd files
$mmdFiles = Get-ChildItem -Path $sourceRoot -Filter "*.mmd" -Recurse

Write-Host "`nFound $($mmdFiles.Count) Mermaid diagrams to render`n" -ForegroundColor Cyan

# Render each diagram
$successCount = 0
$failCount = 0

foreach ($file in $mmdFiles) {
    # Determine output paths
    $relativePath = $file.FullName.Substring($sourceRoot.Length + 1)
    $category = $relativePath.Split('\')[0]
    $baseName = [System.IO.Path]::GetFileNameWithoutExtension($file.Name)
    
    $pngOutput = Join-Path (Join-Path $renderedRoot $category) "$baseName.png"
    $svgOutput = Join-Path (Join-Path $renderedRoot $category) "$baseName.svg"
    
    Write-Host "Rendering: $($file.Name)" -ForegroundColor Yellow
    
    # Render to PNG (high quality, 4K width)
    try {
        & mmdc -i $file.FullName -o $pngOutput -w 3840 -H 2160 -b transparent 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  ✓ PNG: $pngOutput" -ForegroundColor Green
        } else {
            Write-Host "  ✗ PNG failed" -ForegroundColor Red
            $failCount++
            continue
        }
    } catch {
        Write-Host "  ✗ PNG error: $_" -ForegroundColor Red
        $failCount++
        continue
    }
    
    # Render to SVG (vector format)
    try {
        & mmdc -i $file.FullName -o $svgOutput -b transparent 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  ✓ SVG: $svgOutput" -ForegroundColor Green
            $successCount++
        } else {
            Write-Host "  ✗ SVG failed" -ForegroundColor Red
            $failCount++
        }
    } catch {
        Write-Host "  ✗ SVG error: $_" -ForegroundColor Red
        $failCount++
    }
    
    Write-Host ""
}

# Summary
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "Rendering Complete!" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "Successfully rendered: $successCount diagrams" -ForegroundColor Green
Write-Host "Failed: $failCount diagrams" -ForegroundColor Red
Write-Host "`nOutput directory: $renderedRoot" -ForegroundColor Cyan
