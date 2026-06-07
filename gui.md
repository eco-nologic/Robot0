# Plan: Python BLE GUI for GyroBot (ROBOT0)

## 1. Objectif
GUI Python desktop qui se connecte automatiquement au robot via BLE, affiche la télémétrie en temps réel, accumule les données en mémoire et les sauvegarde en CSV sur demande. Envoie les mêmes commandes que le serveur Web existant.

## 2. Fichiers

```
C:\Users\ericc\Documents\VSCODE\ROBOT0\gui\gui.py        <- fichier principal
C:\Users\ericc\Documents\VSCODE\ROBOT0\gui\requirements.txt
C:\Users\ericc\Documents\VSCODE\ROBOT0\gui\Output\       <- créé automatiquement au premier save
```

Template de base : `C:\Users\ericc\Documents\VSCODE\SIMPLE_ROBOT\gui.py` (tkinter + bleak + matplotlib).

## 3. Constantes BLE (depuis BleManager.cpp)

```python
ROBOT_NAME   = "GyroBot_BLE"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_TX_UUID = "87654321-4321-4321-4321-cba987654321"  # Notify — télémétrie robot → GUI
CHAR_RX_UUID = "11111111-2222-3333-4444-555555555555"  # Write  — commandes GUI → robot
```

Les commandes sont des **chaînes UTF-8 simples** (pas JSON) : `"S1"`, `"S2"`, `"S3"`, `"FLECHE"`, `"STOP"`.

## 4. Format CSV (22 champs, depuis Telemetry.cpp `getCSVHeader()`)

```
timestamp,penX,penY,robotHeading,heading,L_steps,R_steps,
accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,
batt,isMoving,targetL,actualV,targetW,actualW
```

## 5. Architecture

Réutilisation complète du pattern du template :

| Composant | Pattern |
|-----------|---------|
| État partagé | `AppState` singleton : `connected`, `client`, `loop`, historiques, buffer, flag `logging` |
| Thread BLE | `asyncio` event loop dans un thread `daemon=True` ; toutes les coroutines BLE via `run_coroutine_threadsafe` |
| Mises à jour UI | `root.after(100, scheduled_update)` — timer 10 Hz sur le thread principal tkinter |
| Callback télémétrie | `on_telemetry(sender, data)` — s'exécute sur le thread BLE, parse CSV, écrit dans `AppState` uniquement, aucun appel UI |
| Auto-connexion | `setup_ble()` lance immédiatement `ble_connect()` au démarrage (pas de bouton manuel). Retry toutes les 5 s si le scan échoue |

## 6. Layout

```
GyroBotGUI (tk.Tk, 1400x900)
  control_bar (top)
    status_label (couleur : gris=scan, orange=connexion, vert=connecté, rouge=déconnecté)
    bouton Disconnect
  content_frame
    left_panel (250px)
      [Séquences]      — boutons S1 / S2 / S3 / FLECHE
      [Arrêt d'Urgence]— grand bouton rouge STOP
      [CSV]            — bouton "Save CSV", label chemin du dernier fichier sauvegardé
      [Télémétrie]     — tk.Text scrollable, paires clé=valeur à 10 Hz
    right_panel (fill)
      matplotlib 2×2
        ax_track   (haut-gauche)  — trajectoire XY + point position actuelle
        ax_compass (haut-droite)  — boussole polaire, flèche cap
        ax_speed   (bas-gauche)   — vitesse roue gauche/droite (time series)
        ax_imu     (bas-droite)   — accélération X/Y/Z (time series)
  status_bar (bottom)  — message d'état + chemin CSV si sauvegardé
```

## 7. Détails d'implémentation

### Auto-connexion
```python
def setup_ble(self):
    self.loop = asyncio.new_event_loop()
    threading.Thread(target=self.loop.run_forever, daemon=True).start()
    asyncio.run_coroutine_threadsafe(self.ble_connect(), self.loop)

async def ble_connect(self):
    while True:
        device = await BleakScanner.find_device_by_name(ROBOT_NAME, timeout=5)
        if device:
            async with BleakClient(device) as client:
                app_state.client = client
                app_state.connected = True
                self.root.after(0, self.update_connection_ui)
                await client.start_notify(CHAR_TX_UUID, self.on_telemetry)
                await asyncio.sleep(float('inf'))  # maintenir la connexion
        await asyncio.sleep(5)  # retry
```

### Envoi de commandes
```python
async def _send(self, text: str):
    await app_state.client.write_gatt_char(CHAR_RX_UUID, text.encode())

def send_command(self, text: str):
    asyncio.run_coroutine_threadsafe(self._send(text), self.loop)
```

### Callback télémétrie
```python
def on_telemetry(self, sender, data: bytearray):
    row = data.decode().strip().split(',')
    if len(row) != 22:
        return
    frame = dict(zip(CSV_HEADER_FIELDS, row))
    # mise à jour des deques (pen, speed, imu)
    app_state.current_telemetry = frame
    if app_state.logging:
        app_state.telemetry_buffer.append(frame)
```

### Sauvegarde CSV (en mémoire → disque sur demande)
- La télémétrie s'accumule dans `app_state.telemetry_buffer` (liste de dicts) dès la connexion
- **Rien n'est écrit sur disque tant que l'utilisateur n'agit pas :**
  - Clic sur **"Save CSV"** dans le panneau gauche
  - Fermeture de la fenêtre (`on_closing()`) — sauvegarde automatique si buffer non vide
- À la sauvegarde : créer `Output/` si absent, écrire `gyrobot_log_YYYYMMDD_HHMMSS.csv` via `csv.DictWriter`, vider le buffer, afficher le chemin dans la status bar
- Le buffer n'est pas vidé entre séquences — l'utilisateur accumule toute la session puis sauvegarde en une fois

## 8. Dépendances

```
# gui/requirements.txt
bleak>=0.21
matplotlib>=3.7
```

## 9. Vérification

1. `python gui/gui.py` → status bar : "Scanning for GyroBot_BLE…"
2. Allumer le robot avec `ENABLE_BLE true` → connexion automatique, label vert
3. Cliquer S1 → robot se déplace, télémétrie se met à jour en temps réel
4. Cliquer STOP → robot s'arrête immédiatement
5. Cliquer "Save CSV" → fichier créé dans `gui/Output/` avec 22 colonnes
6. Couper le robot → GUI passe en rouge, relance le scan automatiquement
7. Fermer la GUI avec buffer non vide → CSV sauvegardé automatiquement
