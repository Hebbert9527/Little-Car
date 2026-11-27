"""Resume a PyPI wheel in bounded HTTP ranges, with all files under this project.
The completed file is accepted only after checking PyPI's SHA256 and byte size.
"""
from pathlib import Path
from datetime import datetime, timezone
from urllib.request import Request, urlopen
from urllib.parse import quote, urlparse
import argparse, hashlib, json, re, shutil, time, zipfile

ROOT = Path(__file__).resolve().parents[1]
HOUSE = ROOT / ".runtime/wheelhouse"
RECOVERED = ROOT / ".runtime/pip-recovered/c_temp_20261118"

def request(url, start=None, end=None):
    headers = {"User-Agent": "Week4-local-dependency-setup/1.0", "Accept-Encoding": "identity"}
    if start is not None: headers["Range"] = f"bytes={start}-{end}"
    return urlopen(Request(url, headers=headers), timeout=30)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("package"); ap.add_argument("version"); ap.add_argument("filename")
    args=ap.parse_args()
    if any(not re.fullmatch(r"[A-Za-z0-9._-]+", x) for x in (args.package,args.version,args.filename)) or not args.filename.endswith(".whl"):
        ap.error("Invalid package/version/filename")
    api=f"https://pypi.org/pypi/{quote(args.package)}/{quote(args.version)}/json"
    with request(api) as response: metadata=json.load(response)
    options=[u for u in metadata["urls"] if u["filename"]==args.filename]
    if len(options)!=1: raise RuntimeError("Expected exactly one matching PyPI wheel")
    item=options[0]; url=item["url"]; size=item["size"]; expected=item["digests"]["sha256"]
    if urlparse(url).scheme!="https" or urlparse(url).hostname!="files.pythonhosted.org":
        raise RuntimeError("Unexpected wheel host")
    HOUSE.mkdir(parents=True,exist_ok=True)
    final=HOUSE/args.filename; part=HOUSE/(args.filename+".part")
    source=HOUSE/(args.filename+".source.json")
    source.write_text(json.dumps(dict(api=api,url=url,size=size,sha256=expected,checked_at_utc=datetime.now(timezone.utc).isoformat()),indent=2),encoding="utf-8")
    if final.exists() and final.stat().st_size==size and hashlib.sha256(final.read_bytes()).hexdigest()==expected:
        print("Already verified:",final,flush=True);return
    candidates=[x for x in RECOVERED.rglob(args.filename) if x.is_file() and 0<x.stat().st_size<size]
    if not part.exists() and candidates:
        candidate=max(candidates,key=lambda x:x.stat().st_size)
        # Confirm the recovered prefix against this exact official distribution.
        count=min(65536,candidate.stat().st_size)
        with request(url,0,count-1) as response:
            if response.status!=206: raise RuntimeError("Range support unavailable")
            prefix=response.read(count+1)
        with candidate.open("rb") as f: old_prefix=f.read(count)
        if prefix==old_prefix:
            shutil.copy2(candidate,part)
            print("Resuming verified prefix:",candidate.stat().st_size,"bytes",flush=True)
    if not part.exists():part.touch()
    if part.stat().st_size>size:raise RuntimeError("Partial file larger than expected wheel")
    failures=0
    while part.stat().st_size<size:
        start=part.stat().st_size; end=min(size-1,start+1024*1024-1)
        try:
            with request(url,start,end) as response:
                expected_range=f"bytes {start}-{end}/{size}"
                if response.status!=206 or response.headers.get("Content-Range")!=expected_range:
                    raise RuntimeError("Wheel server returned an unexpected range")
                got=0
                with part.open("ab") as f:
                    while got<end-start+1:
                        chunk=response.read(min(65536,end-start+1-got))
                        if not chunk:break
                        f.write(chunk);f.flush();got+=len(chunk)
                if got!=end-start+1:raise RuntimeError("Incomplete range; retained for resume")
            failures=0
            print(f"Downloaded {part.stat().st_size}/{size} bytes in D: drive",flush=True)
        except Exception:
            failures+=1
            if failures>=3:raise
            print("Retrying from saved D: partial",part.stat().st_size,flush=True)
            time.sleep(1)
    actual=hashlib.sha256(part.read_bytes()).hexdigest()
    if actual!=expected:raise RuntimeError("Wheel SHA256 mismatch; partial retained for inspection")
    with zipfile.ZipFile(part) as wheel:
        if wheel.testzip() is not None:raise RuntimeError("Wheel CRC verification failed")
    part.replace(final)
    print("Verified SHA256:",actual,flush=True)
    print("WHEEL:",final,flush=True)

if __name__=="__main__":main()
