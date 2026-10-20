# Gelenkgrenzen

Jedes `Joint` besitzt `θmin` und `θmax` in Radiant. Die Grenzen beziehen sich
auf den Controller-Winkel; der Geometrie-Offset `Δϑ` wird erst bei der
Vorwärtskinematik addiert.

Für die Simulation gelten zunächst −π bis +π (−180° bis +180°) pro Gelenk.
Bei einer neuen Datenbank kommen die Werte aus den letzten beiden Spalten
von `CHAIN` in `src/main.rs`. Die Spalten `θmin` und `θmax` werden direkt
mit der Tabelle angelegt. Bestehende Datenbanken werden nicht automatisch migriert.

Bei einer bestehenden Datenbank die Grenzen direkt in `joints` ändern,
zum Beispiel Gelenk 1 auf ±90° begrenzen:

```sql
UPDATE joints
SET θmin = -1.5707963267948966,
    θmax = 1.5707963267948966
WHERE id = 1;
```

Danach das Backend neu starten: Der Solver lädt die Geometrie und Grenzen
beim Start. IDs reichen von 0 bis 5. Die `/configuration`-Antwort zeigt
weiterhin Grad an; die Datenbank und der Solver verwenden Radiant.

Jeder Suchschritt der IK bleibt innerhalb der Grenzen. Gelenke an einer
Grenze können festgehalten werden, während die übrigen Gelenke weiter nach
einer Lösung suchen. Gleichwertige Winkel werden möglichst nahe an der
Startkonfiguration gewählt, ohne die Grenzen zu überschreiten.

Ungültige Grenzen, nicht endliche Werte und Startwinkel außerhalb der
Grenzen werden abgewiesen. Findet die lokale IK keine zulässige Lösung,
wird kein Bewegungsauftrag gesendet. Sie garantiert nicht, jede mögliche
Lösung zu finden. Ein Simulator mit alten Winkelständen außerhalb der
Grenzen muss zusammen mit dem Backend neu gestartet werden.
