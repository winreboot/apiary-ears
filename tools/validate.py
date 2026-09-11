#!/usr/bin/env python3
"""Validate an Apiary Ears acoustic session.

    python tools/validate.py data/*/sessions/*.json

Checks the schema and the things a schema cannot: that band arrays are the right
length, that scores are plausible, and - the one that actually matters - that
events carry notes saying what happened. Exits non-zero so CI can use it.
"""
import json, sys, glob, os

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.join(HERE, "..", "schema", "acoustic-session-v1.schema.json")

def check(path, schema):
    problems, warnings = [], []
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except Exception as e:
        return [f"not valid JSON: {e}"], []

    try:
        import jsonschema
        v = jsonschema.Draft202012Validator(schema)
        for err in sorted(v.iter_errors(d), key=lambda e: e.path):
            loc = "/".join(str(p) for p in err.path) or "(root)"
            problems.append(f"{loc}: {err.message}")
    except ImportError:
        warnings.append("jsonschema not installed - structural check skipped "
                        "(pip install jsonschema)")

    nb = len(d.get("bands_hz") or [])
    mins = d.get("minutes") or []
    evs = d.get("events") or []

    bad = [i for i, m in enumerate(mins) if len(m.get("band_db") or []) != nb]
    if bad:
        problems.append(f"{len(bad)} minute row(s) do not have {nb} band values")
    bad = [i for i, e in enumerate(evs) if len(e.get("band_dev_db") or []) != nb]
    if bad:
        problems.append(f"{len(bad)} event(s) do not have {nb} band deviations")

    ns = len(d.get("nose_steps_c") or [])
    if ns:
        bad = [i for i, e in enumerate(evs)
               if e.get("nose_spec_kohm") is not None and len(e["nose_spec_kohm"]) != ns]
        if bad:
            problems.append(f"{len(bad)} event(s) have a fingerprint that is not {ns} steps long")
        onestep = [i for i, e in enumerate(evs)
                   if e.get("nose_spec_kohm")
                   and len([x for x in e["nose_spec_kohm"] if x]) < ns]
        if onestep:
            warnings.append(f"{len(onestep)} event(s) have an incomplete fingerprint - "
                            "older firmware recorded only some heater steps")

    if not mins and not evs:
        problems.append("neither minutes nor events - nothing to learn from")

    if mins:
        sc = [m.get("score") for m in mins if m.get("score") is not None]
        if sc and max(sc) == min(sc):
            warnings.append("every score is identical - the baseline may not have settled "
                            "before recording started")
        flat = [x for m in mins for x in (m.get("band_db") or [])]
        if flat and max(flat) == min(flat):
            problems.append("every band value is identical - the microphone was not producing audio")

    noted = sum(1 for e in evs if (e.get("note") or "").strip())
    if evs and noted == 0:
        warnings.append(f"none of the {len(evs)} events say what was happening - "
                        "unannotated events are much less useful to anyone else")
    elif evs and noted < len(evs) / 2:
        warnings.append(f"only {noted}/{len(evs)} events are annotated")

    lbl = (d.get("label") or "").strip().lower()
    if lbl in {"test", "todo", "label", "unknown", ""}:
        problems.append(f"label {lbl!r} says nothing")
    if not d.get("score_weights"):
        warnings.append("no score_weights - without them these scores cannot be reproduced")
    return problems, warnings

def main(argv):
    paths = []
    for a in argv or ["data/*/sessions/*.json"]:
        paths.extend(sorted(glob.glob(a)))
    if not paths:
        print("no files matched"); return 1
    with open(SCHEMA, encoding="utf-8") as f:
        schema = json.load(f)
    bad = 0
    for p in paths:
        problems, warnings = check(p, schema)
        if problems:
            bad += 1
            print(f"FAIL {p}")
            for m in problems: print(f"   - {m}")
        else:
            print(f"ok   {p}")
        for m in warnings: print(f"   ~ {m}")
    print(f"\n{len(paths) - bad}/{len(paths)} file(s) valid")
    return 1 if bad else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
