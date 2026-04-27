def _parse_version(version):
    return [int(piece) for piece in version.split(".")]

def _compare_versions(lhs, rhs):
    n = max(len(lhs), len(rhs))
    for i in range(n):
        l = lhs[i] if i < len(lhs) else 0
        r = rhs[i] if i < len(rhs) else 0
        if l < r:
            return -1
        if l > r:
            return 1
    return 0

def parse_requirements(requirements):
    parsed = []
    for requirement in requirements.split(","):
        req = requirement.strip()
        if not req:
            continue

        op = None
        for candidate in ["<=", ">=", "==", "!=", "<", ">"]:
            if req.startswith(candidate):
                op = candidate
                break

        if op == None:
            fail("Invalid version requirement '{req}'.".format(req = req))

        parsed.append((op, _parse_version(req[len(op):].strip())))

    return parsed

def check_all_requirements(version, requirements):
    parsed_version = _parse_version(version)
    for op, required in requirements:
        cmp_result = _compare_versions(parsed_version, required)
        if op == "<" and not cmp_result < 0:
            return False
        if op == "<=" and not cmp_result <= 0:
            return False
        if op == ">" and not cmp_result > 0:
            return False
        if op == ">=" and not cmp_result >= 0:
            return False
        if op == "==" and not cmp_result == 0:
            return False
        if op == "!=" and not cmp_result != 0:
            return False
    return True
