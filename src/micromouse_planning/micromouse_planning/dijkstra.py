#!/usr/bin/env python3
"""Dijkstra = A* s nul-heuristikom. Isti optimalni put kao A*, ali slijepa
pretraga (bez vuce prema cilju) prosiri vise cvorova. Nasljeduje svu logiku
iz AStar; jedina razlika je heuristika = 0. Postoji za usporedbu broja
pregledanih cvorova protiv A*."""

from micromouse_planning.a_star import AStar


class Dijkstra(AStar):
    def _heuristic(self, x, y):
        return 0.0
