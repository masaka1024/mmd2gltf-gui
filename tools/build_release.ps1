# Windows EXE 版のビルドから配布 zip までを一括で行う。
#
#   powershell -ExecutionPolicy Bypass -File tools\build_release.ps1 -Version v1.3.0
#   powershell -ExecutionPolicy Bypass -File tools\build_release.ps1 -Version v1.3.0 -Publish
#
# -Publish を付けると、タグを打って GitHub リリースを作るところまで行う。
# 付けなければ zip を作るだけ (既定)。
#
# 実際の検査とパッケージングは tools\release_pack.py が行う。
# 過去の出荷事故 (版数の更新漏れ / 実行ファイル名の誤記 / zip の日本語名破損 /
# 古い .pyc の混入) は、そちらの検査で落ちる。
param(
    [Parameter(Mandatory = $true)][string]$Version,
    [switch]$Publish,
    [string]$NotesFile
)

$ErrorActionPreference = "Stop"

# ★$PSScriptRoot は param() の既定値の中では使えないので、ここで解決する。
$repo = Split-Path -Parent $PSScriptRoot

if ($Version -notmatch '^v\d+\.\d+\.\d+$') {
    throw "-Version は v1.3.0 の形式で指定してください (指定値: $Version)"
}
# タグは大文字 V、リリースのタイトルは小文字 v という既存の慣習に合わせる。
$tag = "V" + $Version.Substring(1)
$zipName = "mmd2gltf-gui-$tag-win64.zip"
$zipPath = Join-Path $repo "dist\release\$zipName"

Write-Output "リポジトリ : $repo"
Write-Output "バージョン : $Version  (タグ $tag)"

# --- 1. 古い成果物を消す -------------------------------------------------
# ★spec は mmd2gltf ディレクトリを datas でまるごと同梱するため、
#   __pycache__ を残すと古い .pyc がそのまま exe に入る。
Write-Output "`n=== [1/4] 掃除 ==="
Get-ChildItem -Path (Join-Path $repo "mmd2gltf") -Filter "__pycache__" -Recurse -Directory -ErrorAction SilentlyContinue |
    ForEach-Object { Write-Output "  rm $($_.FullName)"; Remove-Item -Recurse -Force $_.FullName }
foreach ($d in @("build", "dist\mmd2gltf.exe")) {
    $p = Join-Path $repo $d
    if (Test-Path $p) { Write-Output "  rm $p"; Remove-Item -Recurse -Force $p }
}

# --- 2. PyInstaller ------------------------------------------------------
Write-Output "`n=== [2/4] PyInstaller ==="
Push-Location $repo
try {
    & python -m PyInstaller --noconfirm --clean "mmd2gltf.spec"
    if ($LASTEXITCODE -ne 0) { throw "PyInstaller が失敗しました (exit $LASTEXITCODE)" }
}
finally { Pop-Location }

# --- 3. 検査 + zip -------------------------------------------------------
Write-Output "`n=== [3/4] 検査と zip ==="
& python (Join-Path $PSScriptRoot "release_pack.py") --version $Version --root $repo
if ($LASTEXITCODE -ne 0) { throw "検査に失敗したのでリリースを中止します (exit $LASTEXITCODE)" }

if (-not (Test-Path $zipPath)) { throw "zip が見つかりません: $zipPath" }

# --- 4. 公開 (任意) ------------------------------------------------------
Write-Output "`n=== [4/4] 公開 ==="
if (-not $Publish) {
    Write-Output "  -Publish が無いので zip の作成までで終了します。"
    Write-Output "  成果物: $zipPath"
    Write-Output "`n  公開するには:"
    Write-Output "    powershell -ExecutionPolicy Bypass -File tools\build_release.ps1 -Version $Version -Publish"
    exit 0
}

$gh = "C:\Program Files\GitHub CLI\gh.exe"
if (-not (Test-Path $gh)) {
    $cmd = Get-Command gh -ErrorAction SilentlyContinue
    if ($cmd) { $gh = $cmd.Source } else { throw "gh CLI が見つかりません" }
}

Push-Location $repo
try {
    # 作業ツリーが汚れたまま公開すると、出荷物とコミットが食い違う。
    $dirty = & git status --porcelain
    if ($dirty) {
        Write-Output $dirty
        throw "作業ツリーに未コミットの変更があります。コミットしてから公開してください。"
    }

    $behind = & git rev-list --count "origin/main..HEAD"
    if ($LASTEXITCODE -eq 0 -and $behind -ne "0") {
        throw "push されていないコミットが $behind 件あります。先に git push してください。"
    }

    $existing = & git tag --list $tag
    if (-not $existing) {
        Write-Output "  タグ $tag を作成"
        & git tag -a $tag -m $Version
        & git push origin $tag
        if ($LASTEXITCODE -ne 0) { throw "タグの push に失敗しました" }
    }
    else { Write-Output "  タグ $tag は既にあります" }

    # ★$args は PowerShell の自動変数なので使わない。
    $ghArgs = @("release", "create", $tag, $zipPath, "--title", $Version, "--latest")
    if ($NotesFile) { $ghArgs += @("--notes-file", $NotesFile) }
    else { $ghArgs += @("--generate-notes") }

    & $gh @ghArgs
    if ($LASTEXITCODE -ne 0) { throw "gh release create に失敗しました" }
}
finally { Pop-Location }

Write-Output "`n完了しました。"
