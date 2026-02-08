# Diagram Rendering Guide

## Quick Render

To render all diagrams at once:
```powershell
cd c:\Users\ratan\Desktop\End-to-End-RL-agent-UAV-flight-controller\WBS\diagrams
.\render_all.ps1
```

## Individual Diagram Rendering

To render a single diagram:
```powershell
# PNG (4K resolution, transparent background)
mmdc -i source\architecture\D1.1_system_context.mmd -o rendered\architecture\D1.1_system_context.png -w 3840 -H 2160 -b transparent

# SVG (vector format, transparent background)
mmdc -i source\architecture\D1.1_system_context.mmd -o rendered\architecture\D1.1_system_context.svg -b transparent
```

## Output Specifications

### PNG Output
- **Resolution**: 3840 x 2160 (4K)
- **Background**: Transparent
- **Format**: PNG-24
- **Use case**: Presentations, documents, high-quality prints

### SVG Output
- **Format**: Scalable Vector Graphics
- **Background**: Transparent
- **Use case**: Web integration, infinite zoom, small file size

## Viewing Rendered Diagrams

### In VS Code
1. Navigate to `diagrams/rendered/`
2. Click on any `.png` or `.svg` file
3. VS Code will display it inline

### In File Explorer
```powershell
# Open rendered directory
explorer c:\Users\ratan\Desktop\End-to-End-RL-agent-UAV-flight-controller\WBS\diagrams\rendered
```

### In Browser
```powershell
# Open specific SVG in default browser
start rendered\architecture\D1.1_system_context.svg
```

## Batch Operations

### Re-render only specific category
```powershell
# Architecture diagrams only
Get-ChildItem source\architecture\*.mmd | ForEach-Object {
    $base = [System.IO.Path]::GetFileNameWithoutExtension($_.Name)
    mmdc -i $_.FullName -o "rendered\architecture\$base.png" -w 3840 -H 2160 -b transparent
}
```

### Export for presentations
```powershell
# Create a presentation/ folder with all PNGs
New-Item -ItemType Directory -Path presentation -Force
Get-ChildItem rendered -Recurse -Filter "*.png" | Copy-Item -Destination presentation\
```

## Troubleshooting

### mmdc not found
```powershell
# Install mermaid-cli globally
npm install -g @mermaid-js/mermaid-cli
```

### Rendering fails for complex diagrams
- Some large diagrams may fail SVG rendering but succeed with PNG
- Try reducing diagram complexity or splitting into multiple diagrams
- Increase timeout: `mmdc -i input.mmd -o output.png -t 60000`

### Output is blurry
- Increase resolution: `-w 5120 -H 2880` (5K)
- For print: `-w 7680 -H 4320` (8K)

## Integration into WBS Documents

To embed diagrams in markdown:
```markdown
![D1.1 System Context](diagrams/rendered/architecture/D1.1_system_context.png)
```

For SVG (better for web):
```markdown
![D1.1 System Context](diagrams/rendered/architecture/D1.1_system_context.svg)
```

## Automated Re-rendering

The `render_all.ps1` script automatically:
1. Creates directory structure if missing
2. Finds all `.mmd` files in `source/`
3. Renders to both PNG and SVG
4. Preserves category organization
5. Provides progress feedback
6. Summarizes success/failure counts

Run it whenever you update diagram source files.
