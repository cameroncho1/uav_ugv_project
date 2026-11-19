"""Export saved verdicts without re-evaluating source recordings."""

from .files import write_evaluation


def report(source, output):
    from .summary import report as build_report

    return build_report(source, output)


__all__ = ["report", "write_evaluation"]
