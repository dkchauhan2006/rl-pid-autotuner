"""
data_source.py  —  get the rig step-test logs straight from the GitHub repo.

    from data_source import fetch_step_tests
    paths = fetch_step_tests()            # {"2V": ".../samplerun_2V.txt", ...}

1. Lists data/raw/ of the repo with the GitHub API and downloads every samplerun_*.txt (raw.githubusercontent.com).
2. If the listing is blocked (API rate limit / firewall), downloads the known file names directly.
3. Files are cached in data/github_cache/<branch>/ ; --refresh re-downloads them.
4. If GitHub cannot be reached at all, falls back to the local copy in data/raw/ (and says so).
"""
import json
import os
import re
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = "dkchauhan2006/rl-pid-autotuner"
BRANCH = "main"
FOLDER = "data/raw"
KNOWN = ["samplerun_2V.txt", "samplerun_3V.txt", "samplerun_4V.txt", "samplerun_5V.txt"]
PATTERN = re.compile(r"^samplerun_(\d+(?:\.\d+)?V)\.txt$")


def _get(url, timeout=30):
    req = urllib.request.Request(url, headers={"User-Agent": "rl-pid-autotuner"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def list_remote(repo=REPO, branch=BRANCH, folder=FOLDER):
    """File names in the GitHub folder (needs the GitHub API)."""
    url = f"https://api.github.com/repos/{repo}/contents/{folder}?ref={branch}"
    items = json.loads(_get(url))
    return [it["name"] for it in items if it.get("type") == "file" and PATTERN.match(it["name"])]


def fetch_step_tests(repo=REPO, branch=BRANCH, folder=FOLDER, refresh=False, verbose=True):
    """Return {run name: local path}, e.g. {"2V": ".../samplerun_2V.txt"}."""
    cache = os.path.join(HERE, "data", "github_cache", branch)
    os.makedirs(cache, exist_ok=True)
    try:
        names = list_remote(repo, branch, folder)
        how = "GitHub API listing"
    except Exception as ex:                                   # rate limit / no API access
        names, how = KNOWN, f"known file names (listing failed: {ex.__class__.__name__})"
    out, failed = {}, []
    for name in names:
        dst = os.path.join(cache, name)
        if refresh or not os.path.exists(dst) or os.path.getsize(dst) == 0:
            try:
                data = _get(f"https://raw.githubusercontent.com/{repo}/{branch}/{folder}/{name}")
                with open(dst, "wb") as f:
                    f.write(data)
            except Exception as ex:
                failed.append((name, ex.__class__.__name__))
                continue
        out[PATTERN.match(name).group(1)] = dst
    if not out:                                               # offline: use the copy that ships with the repo
        local = os.path.join(HERE, "data", "raw")
        out = {PATTERN.match(n).group(1): os.path.join(local, n) for n in sorted(os.listdir(local)) if PATTERN.match(n)}
        if verbose:
            print(f"GitHub not reachable ({failed}); using local files in {local}")
        return out
    if verbose:
        print(f"Step tests from github.com/{repo}/{folder} @ {branch} ({how}): "
              + ", ".join(f"{k} ({os.path.getsize(v) // 1024} kB)" for k, v in sorted(out.items())))
        for n, e in failed:
            print(f"  ! could not download {n}: {e}")
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--branch", default=BRANCH)
    ap.add_argument("--refresh", action="store_true")
    a = ap.parse_args()
    print(fetch_step_tests(branch=a.branch, refresh=a.refresh))
