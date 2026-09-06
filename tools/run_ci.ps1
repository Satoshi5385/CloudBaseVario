[CmdletBinding()]
param(
    [string]$Image = "espressif/idf:v6.0.2",
    [ValidateSet("python-tests", "python-compile", "configure", "host-c", "firmware", "configuration", "all")]
    [string]$Stage = "all"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "docker.exe was not found. Install Docker Desktop first."
}

& docker info --format "{{.ServerVersion}}" | Out-Null
if ($LASTEXITCODE -ne 0) {
    throw "Docker Engine is unavailable. Start Docker Desktop in Linux containers mode and wait until it is ready."
}

$dockerArguments = @(
    "run"
    "--rm"
    "--init"
    "--env", "CI=true"
    "--env", "GITHUB_ACTIONS=true"
    "--env", "GITHUB_WORKSPACE=/workspace"
    "--env", "CCACHE_DIR=/workspace/.ccache"
    "--env", "CCACHE_TEMPDIR=/workspace/.ccache-tmp"
    "--env", "CI_BUILD_DIR=build-ci"
    "--volume", "${repoRoot}:/workspace"
    "--workdir", "/workspace"
    "--entrypoint", "/bin/bash"
    $Image
    "ci/run_ci.sh"
    $Stage
)

Write-Host "Running CI stage '$Stage' in the GitHub Actions ESP-IDF container: $Image"
& docker @dockerArguments
if ($LASTEXITCODE -ne 0) {
    throw "Local CI failed (exit code: $LASTEXITCODE)."
}

Write-Host "All local CI checks passed."
