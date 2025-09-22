# Spécification du système robotique ULTIMA

Ce document décrit la spécification technique du système robotique **ULTIMA**.  
Il couvre les aspects de communication, perception, navigation, contrôle et supervision.

> **Note :**  
> Pour l'instant, seule la partie **Communication** est renseignée.  
> Les autres sections sont présentes à titre de planification et seront complétées au fur et à mesure du développement.

---

## Sommaire

1. [Introduction](#1-introduction)  
2. [Architecture système](#2-architecture-système)  
3. [Communication](#3-communication) *(complété)*  
    - 3.1 Objectifs  
    - 3.2 Contraintes réseau  
    - 3.3 Protocole série  
    - 3.4 Types de trames  
    - 3.5 ROS 2 Topics et Interfaces  
    - 3.6 Gestion des erreurs  
4. [Perception](#4-perception) *(à venir)*  
    - 4.1 Capteurs  
    - 4.2 IMU  
    - 4.3 LiDAR / PointCloud  
    - 4.4 Vision (caméras, détection d'obstacles, etc.)  
5. [Navigation](#5-navigation) *(à venir)*  
    - 5.1 SLAM  
    - 5.2 Planification de trajectoire  
    - 5.3 Évitement d'obstacles  
6. [Contrôle](#6-contrôle) *(à venir)*  
    - 6.1 Asservissement moteur  
    - 6.2 Pan/Tilt camera  
    - 6.3 Propulsion et Steering  
7. [Supervision et diagnostic](#7-supervision-et-diagnostic) *(à venir)*  
    - 7.1 Health monitoring  
    - 7.2 Logs ROS2 et gestion des alertes  
8. [Annexes](#8-annexes)  
    - 8.1 Glossaire  
    - 8.2 Révisions et historique

---

## 1. Introduction
Le projet **ULTIMA** est un système robotique composé :
- D'un **rover mobile** équipé d'une Jetson Nano (Ubuntu 18.04, ROS2 Eloquent, architecture ARM).
- D'une **station fixe** sous Ubuntu 24.04 (ROS2 Humble, x64).

L'objectif global est d'assurer la **navigation autonome** du rover avec une supervision en temps réel depuis la station.

---

## 2. Architecture système
*À compléter ultérieurement.*

---

## 3. Communication
Cette section définit les règles et protocoles pour la communication entre la station et le rover.  
L'objectif principal est de garantir un **échange fiable et efficace**, avec une bande passante limitée à **35 kbps** via une liaison série XBee.

### 3.1 Objectifs
- Transmettre les commandes de contrôle en temps réel (100 Hz cible).
- Superviser la liaison avec des pings et métriques (latence, pertes).
- Supporter des commandes critiques comme **reset global** et **init actuateurs**.
- Préparer l'extension future pour la télémétrie (IMU, LiDAR).

---

### 3.2 Contraintes réseau
| Paramètre                  | Valeur |
|----------------------------|--------|
| Bande passante max          | 35 kbps |
| Baudrate XBee                | 115200 bauds |
| Timeout réception            | 1000 ms |
| Fréquence ping supervision   | 250 ms |
| Fréquence max commandes      | 100 Hz (réductible à 75, 50 ou 25 Hz si nécessaire) |

---

### 3.3 Protocole série

Chaque trame échangée suit la structure suivante :

| Champ        | Taille (octets) | Description |
|--------------|----------------|-------------|
| Start Code   | 1              | `0x55` (début de trame) |
| Compteur     | 2              | Compteur cyclique, big-endian |
| Header       | 1              | Type de trame (commande, supervision, etc.) |
| Subheader    | 1              | Détail précis du type |
| Taille       | 1              | Taille du **payload uniquement** |
| Payload      | Variable       | Données selon le type |
| Checksum     | 1              | Somme mod 256 de tous les octets précédents |

#### Types de Header
| Header | Signification |
|--------|---------------|
| 0x01   | Commande mobilité |
| 0x02   | Supervision |
| 0x03   | Futur capteurs (IMU, LiDAR) |

#### Types de Subheader (pour Header 0x01 : Commandes mobilité)
| Subheader | Commande |
|-----------|----------|
| 0x01 | Pan/Tilt |
| 0x02 | Steering + Propulsion |

#### Types de Subheader (pour Header 0x02 : Supervision)
| Subheader | Action |
|-----------|--------|
| 0x01 | Ping |
| 0x02 | Reset global robot |
| 0x03 | Init actuateurs |

---

### 3.4 Format des payloads

#### Pan / Tilt
- 2 valeurs `int16` signées (-180 à +180 degrés)
- Taille totale : **4 octets**

#### Steering / Propulsion
- Steering : `int8` (-100° à +100°)  
- Propulsion : `int8` (-100% à +100%)
- Taille totale : **2 octets**

---

### 3.5 ROS 2 Topics et Interfaces

#### Station
| Topic | Type ROS | Direction | Description |
|-------|----------|-----------|-------------|
| `/rover/command/pan_tilt` | `std_msgs/Int16MultiArray` | Publish → Rover | Incréments pan & tilt |
| `/rover/command/steering_propulsion` | `std_msgs/Int8MultiArray` | Publish → Rover | Steering et propulsion |
| `/rover/status/supervision` | `custom_msgs/SupervisionStatus` | Subscribe ← Rover | Latence, pertes, état liaison |

#### Rover
| Topic | Type ROS | Direction | Description |
|-------|----------|-----------|-------------|
| `/actuator/pan_tilt` | `std_msgs/Int16MultiArray` | Publish interne | Commande envoyée aux actuateurs |
| `/actuator/drive` | `std_msgs/Int8MultiArray` | Publish interne | Steering + propulsion |

---

### 3.6 Gestion des erreurs
- Si latence > 200 ms pendant 10 secondes → arrêt des envois de commandes.
- Tentative de reconnexion série toutes les **10 s**.
- Logs ROS2 :
  - `INFO` : Latence moyenne et % pertes (mis à jour chaque seconde).
  - `ERROR` : Liaison perdue ou reset global reçu.

---

## 4. Perception *(à venir)*
Cette section décrira les capteurs du rover et les flux associés :
- IMU
- LiDAR 1D type RPLidar
- Caméras

---

## 5. Navigation *(à venir)*
- SLAM
- Planification trajectoire
- Évitement obstacles

---

## 6. Contrôle *(à venir)*
- Asservissements moteurs
- Gestion du Pan/Tilt
- Contrôle de propulsion et steering

---

## 7. Supervision et diagnostic *(à venir)*
- Monitoring de santé du système
- Gestion des logs et alertes
- Interface de contrôle côté station

---

## 8. Annexes

### 8.1 Glossaire
| Terme | Définition |
|-------|------------|
| ROS2 | Robot Operating System 2 |
| XBee | Module de communication sans fil série |

### 8.2 Historique des révisions
| Version | Date | Auteur | Description |
|----------|------|--------|-------------|
| 0.1 | 2025-09-21 | Initiale | Première version, partie communication remplie |

