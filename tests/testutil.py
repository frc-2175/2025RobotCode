def expecteq(actual, expected):
  if actual != expected:
    raise RuntimeError(f"expected {expected} but got {actual}")

def expecterror(f, error=Exception):
  try:
    f()
  except error:
    return
  except Exception as e:
    raise e
  raise RuntimeError(f"expected function to raise {error.__name__}, but no exception was thrown")
