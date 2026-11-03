"""Robot discovery must ignore topics that only have subscribers."""

from __future__ import annotations

from ros2_robot_interface.utils.discovery import filter_topics_with_publishers


def test_keeps_only_topics_with_publishers():
    counts = {
        "/robot_description": 0,
        "/g1d/robot_description": 1,
    }
    assert filter_topics_with_publishers(
        ["/robot_description", "/g1d/robot_description"], counts.get
    ) == ["/g1d/robot_description"]


def test_keeps_topic_when_the_query_fails():
    def boom(_topic):
        raise RuntimeError("graph unavailable")

    assert filter_topics_with_publishers(["/g1d/robot_description"], boom) == [
        "/g1d/robot_description"
    ]


def test_empty_input_stays_empty():
    assert filter_topics_with_publishers([], lambda _t: 0) == []
