from tram_model.slip import SlipConfig, SlipDetector


def _detector():
    return SlipDetector(SlipConfig(
        disagreement=0.6, innovation_limit=1.0, speed_cap=26.0))


def test_agreeing_bogies_override_model():
    det = _detector()
    front = det.feed("front", 0.0, 4.0, 0.0, 0.5, 7.0)
    rear = det.feed("rear", 0.05, 4.0, 0.0, 0.5, 7.0)
    assert front.speed == 4.0
    assert rear.speed == 4.0
    assert rear.slip is False


def test_spinning_bogie_is_dropped():
    det = _detector()
    det.feed("front", 0.0, 4.0, 4.0, 0.2, 7.0)
    det.feed("rear", 0.05, 4.0, 4.0, 0.2, 7.0)
    bad = det.feed("front", 0.10, 8.0, 4.0, 0.2, 7.0)
    assert bad.speed is None
    assert bad.slip is True
    assert bad.kind == "slip"
    good = det.feed("rear", 0.15, 4.1, 4.0, 0.2, 7.0)
    assert good.speed is not None
