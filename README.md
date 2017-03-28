# 3DRender

## Setup
1. Eclipse installieren
2. *Existing Project from Workspace* importieren
3. *Java Application* Run Configuration hinzufügen
  1. Main Class `sim.CarbotSim`
  2. Programm Arguments `-e ${workspace_loc:3DRender}/nav_environment2.txt -c democontroller.Navigate`
4. Ausführen

## ToDo

- [ ] Texturen für TriangleMesh
- [ ] Lichtquelle positionieren und Schatten entsprechend der Lichtquellenposition nach dem Roth verfahren nutzen
- [ ] Reifenspuren zeichnen
- [ ] Himmel und Boden mit Texturen zeichen (evtl. SubScene oder 2D-Grafik)
- [ ] WireFrame-Ansicht (trivial)
- [ ] Optimieren und dem Code eine saubere Struktur versehen
