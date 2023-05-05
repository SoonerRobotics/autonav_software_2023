def hash(s: str):
    h = 5381
    c = 0

    for b in range(len(s)):
        c = ord(s[b])
        h = ((h << 5) + h) + c

    return h & 0xFFFFFFFFFFFF


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
