"""Reserved interface; the old endpoint sequentializer is not queue refinement."""


def run(*args, **kwargs):
    raise NotImplementedError('Queue sequentialization is not implemented; use --refinement monitor.')
