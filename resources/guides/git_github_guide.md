# Git & GitHub Guide für das mr_SHERPA7000 Projekt

## Inhaltsverzeichnis
1. [Einführung](#einführung)
2. [GitHub-Konto erstellen](#github-konto-erstellen)
3. [SSH-Key einrichten](#ssh-key-einrichten)
4. [Git installieren](#git-installieren)
5. [Repository klonen](#repository-klonen)
6. [Grundlegende Git-Operationen](#grundlegende-git-operationen)
7. [Branch-Strategie](#branch-strategie)
8. [Kollaboratives Arbeiten](#kollaboratives-arbeiten)
9. [Konfliktlösung](#konfliktlösung)
10. [Best Practices](#best-practices)
11. [Häufige Fehler und Lösungen](#häufige-fehler-und-lösungen)

## Einführung

Git ist ein Versionskontrollsystem zur Nachverfolgung von Änderungen im Quellcode. GitHub ist die Plattform, auf der unser Code gehostet wird.

Im mr_SHERPA7000 Projekt verwenden wir folgendes Repository:
```
git@github.com:Master-D1990/mr_SHERPA7000.git
```

Die Hauptbestandteile unserer Arbeit:
- **Main Branch**: Stabiler, produktionsfähiger Code
- **Develop Branch**: Aktive Entwicklung und Integration neuer Funktionen
- **Feature Branches**: Größere Änderungen, die parallel zur Hauptentwicklung stattfinden

**WICHTIG:** Vor Arbeitsbeginn:
1. GitHub-Konto erstellen
2. SSH-Key generieren
3. Dave (Master-D1990) kontaktieren, um als Collaborator hinzugefügt zu werden

## GitHub-Konto erstellen

### Neue Benutzer
1. [GitHub](https://github.com) besuchen
2. "Sign up" klicken und Anweisungen folgen
3. Benutzernamen wählen, E-Mail-Adresse eingeben und sicheres Passwort erstellen
4. E-Mail-Adresse bestätigen

### Nach der Kontoerstellung
1. **Wichtig:** Dave (Master-D1990) per E-Mail oder persönlich kontaktieren und folgende Informationen mitteilen:
   - GitHub-Benutzernamen
   - Namen und Rolle im Projekt
   - Öffentlichen SSH-Key (siehe nächster Abschnitt)
   
2. Ohne diese Mitteilung an Dave keine Arbeit am Repository möglich!

3. Dave fügt dich als Collaborator zum Repository hinzu und registriert deinen SSH-Key

4. Du erhältst eine Einladungs-E-Mail von GitHub, die akzeptiert werden muss

## SSH-Key einrichten

SSH-Keys ermöglichen sichere Authentifizierung ohne Passwort-Eingabe bei jeder Operation.

### SSH-Key generieren (Windows, macOS, Linux)

1. Terminal öffnen (oder Git Bash unter Windows)
2. SSH-Key generieren mit:
   ```bash
   ssh-keygen -t ed25519 -C "deine-email@beispiel.de"
   ```
3. Standard-Speicherort für den Key bestätigen
4. Optional: Passphrase eingeben (empfohlen für zusätzliche Sicherheit)

### SSH-Key zu GitHub hinzufügen

1. Öffentlichen SSH-Key kopieren:
   ```bash
   # Unter macOS
   cat ~/.ssh/id_ed25519.pub | pbcopy
   
   # Unter Windows (Git Bash)
   cat ~/.ssh/id_ed25519.pub | clip
   
   # Unter Linux
   cat ~/.ssh/id_ed25519.pub | xclip -selection clipboard
   ```
   
   Alternativ: Datei `~/.ssh/id_ed25519.pub` mit Texteditor öffnen und Inhalt kopieren.

2. Auf GitHub zu Einstellungen gehen (rechts oben auf Profilbild klicken, dann "Settings")
3. Im linken Menü auf "SSH and GPG keys" klicken
4. Auf "New SSH key" klicken
5. Aussagekräftigen Titel eingeben (z.B. "Arbeits-Laptop")
6. Kopierten öffentlichen Schlüssel in "Key"-Feld einfügen
7. Auf "Add SSH key" klicken
8. GitHub-Passwort bestätigen, wenn dazu aufgefordert

### SSH-Verbindung testen

```bash
ssh -T git@github.com
```

Bei erfolgreicher Einrichtung erscheint eine Begrüßungsnachricht.

## Git installieren

### Windows
1. Git-Installer von [https://git-scm.com](https://git-scm.com) herunterladen
2. Installer ausführen und Anweisungen folgen
3. Standardeinstellungen sind für die meisten Benutzer geeignet

### macOS
```bash
# Mit Homebrew
brew install git

# Alternativ: Installer von git-scm.com verwenden
```

### Linux (Ubuntu/Debian)
```bash
sudo apt update
sudo apt install git
```

### Git konfigurieren
```bash
git config --global user.name "Dein Name"
git config --global user.email "deine-email@beispiel.de"
```

## Repository klonen

### Einrichtung des Sherpa-Ordners

```bash
# Im Home-Verzeichnis oder wo der catkin_ws liegt:
cd ~

# Falls catkin_ws noch nicht existiert:
mkdir -p catkin_ws

# In catkin_ws wechseln:
cd catkin_ws

# Sherpa-Verzeichnis erstellen:
mkdir -p Sherpa

# In das Sherpa-Verzeichnis wechseln:
cd Sherpa

# Repository als src-Ordner klonen:
git clone git@github.com:Master-D1990/mr_SHERPA7000.git src
```

Nach diesem Befehl wird ein `src`-Ordner im Sherpa-Verzeichnis erstellt, der alle Dateien aus dem Repository enthält.

### Wichtig: Wo Git-Befehle ausführen

Git-Befehle wie `git status`, `git pull` oder `git commit` müssen immer im `src`-Ordner ausgeführt werden, nicht im übergeordneten Sherpa-Verzeichnis:

```bash
# In den src-Ordner wechseln, um Git-Befehle auszuführen:
cd ~/catkin_ws/Sherpa/src

# Jetzt können Git-Befehle verwendet werden:
git status
git branch
# usw.
```

### Alternative: Falls der src-Ordner bereits existiert

```bash
# Im Sherpa-Verzeichnis:
cd ~/catkin_ws/Sherpa/src

# Repository in aktuelles Verzeichnis klonen:
git clone git@github.com:Master-D1990/mr_SHERPA7000.git .
```

## Grundlegende Git-Operationen

### Wichtig: Git-Befehle immer im src-Ordner ausführen

Alle Git-Befehle müssen im src-Ordner ausgeführt werden, nicht im übergeordneten Sherpa-Verzeichnis:

```bash
# In den src-Ordner wechseln:
cd ~/catkin_ws/Sherpa/src

# Jetzt können Git-Befehle verwendet werden
```

Wenn du versuchst, Git-Befehle im Sherpa-Verzeichnis auszuführen, erhältst du einen Fehler wie:
```
fatal: not a git repository (or any of the parent directories): .git
```

### Status überprüfen

```bash
# Im src-Ordner ausführen:
git status
```
Zeigt den aktuellen Status des Repositorys an.

### Branches anzeigen

```bash
git branch               # Lokale Branches
git branch -a            # Alle Branches (inkl. Remote)
```

### Branch wechseln

```bash
git checkout develop     # Zum develop-Branch wechseln
```

### Neuen Branch erstellen

```bash
git checkout -b feature/neue-funktion
```

### Änderungen anzeigen

```bash
git diff                 # Noch nicht vorgemerkte Änderungen
```

### Für Commit vormerken

```bash
git add dateiname.txt    # Bestimmte Datei
git add .                # Alle Änderungen
```

### Commit erstellen

```bash
git commit -m "Beschreibung der Änderungen"
```

### Änderungen hochladen

```bash
git push origin feature/neue-funktion
```

### Änderungen herunterladen

```bash
git pull origin develop
```

## Branch-Strategie

### Main Branch
- Stabiler Code
- Keine direkten Commits
- Nur über geprüfte Pull Requests aktualisieren

### Develop Branch
- Aktive Entwicklung
- Enthält neueste Funktionen für nächsten Release

### Feature Branches
- Für neue Funktionen oder größere Änderungen
- Von develop abzweigen
- Namenskonvention: `feature/beschreibende-bezeichnung`

### Bugfix Branches
- Für Fehlerbehebungen
- Namenskonvention: `bugfix/beschreibende-bezeichnung`

### Workflow für neue Funktionen

1. Develop-Branch aktualisieren:
   ```bash
   git checkout develop
   git pull origin develop
   ```

2. Feature-Branch erstellen:
   ```bash
   git checkout -b feature/neue-funktion
   ```

3. Änderungen committen und pushen:
   ```bash
   git add .
   git commit -m "Beschreibung"
   git push origin feature/neue-funktion
   ```

4. Pull Request auf GitHub erstellen

## Kollaboratives Arbeiten

### Pull Requests erstellen

1. Auf GitHub zum Repository gehen
2. Zum eigenen Branch wechseln
3. "Pull request" klicken
4. Develop als Zielbranch wählen
5. Beschreibung hinzufügen
6. "Create pull request" klicken

### Code-Reviews

1. Reviewers erhalten Benachrichtigung
2. Können Kommentare hinzufügen und PR genehmigen

## Konfliktlösung

### Konflikte bei Pull oder Merge

1. Git zeigt betroffene Dateien an
2. Dateien öffnen und Konflikte lösen (Markierungen entfernen):
   ```
   <<<<<<< HEAD
   Deine Änderungen
   =======
   Änderungen aus dem anderen Branch
   >>>>>>> feature/andere-funktion
   ```

3. Änderungen committen:
   ```bash
   git add <dateien>
   git commit
   git push
   ```

## Best Practices

### Commit-Nachrichten
- Präzise, beschreibende Nachrichten
- Mit Verb beginnen (Add, Fix, Update)
- Kurze erste Zeile, dann Details

### Regelmäßiges Synchronisieren
```bash
git checkout develop
git pull origin develop
git checkout feature/deine-funktion
git merge develop
```

### Tagging wichtiger Versionen
```bash
git tag -a v1.0.0 -m "Version 1.0.0"
git push origin v1.0.0
```

## Häufige Fehler und Lösungen

### "Failed to push some refs"
```bash
git pull origin <branch-name>
# Konflikte lösen
git push origin <branch-name>
```

### "Your branch is behind 'origin/develop'"
```bash
git pull origin develop
```

### Commits rückgängig machen
```bash
git reset --soft HEAD~1    # Änderungen beibehalten
git reset --hard HEAD~1    # Änderungen verwerfen
```

### Dateien zurücksetzen
```bash
git checkout -- <dateiname>
```

---