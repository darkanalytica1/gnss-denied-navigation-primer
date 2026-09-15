import os

from gnssdenied import demo


def test_demo_writes_csv_without_plot(tmp_path, capsys):
    assert demo.main(["--out", str(tmp_path), "--no-plot", "--duration", "360"]) == 0
    for name in ("summary.csv", "timeseries.csv", "fixes.csv"):
        assert os.path.getsize(tmp_path / name) > 0
    out = capsys.readouterr().out
    assert "VIO + VPR, gated" in out


def test_demo_png_when_matplotlib_available(tmp_path):
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        return
    assert demo.main(["--out", str(tmp_path), "--duration", "360"]) == 0
    assert os.path.getsize(tmp_path / "gnssdenied_demo.png") > 10_000
