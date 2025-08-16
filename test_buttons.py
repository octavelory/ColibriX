#!/usr/bin/env python3
import pygame
import sys
import time

# --- COULEURS POUR L'AFFICHAGE ---
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    GRAY = '\033[90m'

def clear_screen():
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

def print_banner():
    clear_screen()
    banner = f"""
{Colors.CYAN}{Colors.BOLD}╔════════════════════════════════════════════════════════════════════════════════╗
║                         🎮 TEST BOUTONS MANETTE 🎮                            ║
╚════════════════════════════════════════════════════════════════════════════════╝{Colors.RESET}

{Colors.YELLOW}Ce script affiche en temps réel les boutons pressés et les axes utilisés{Colors.RESET}
{Colors.GREEN}Utilisez votre manette pour voir les événements s'afficher ci-dessous{Colors.RESET}
{Colors.RED}Appuyez sur Ctrl+C pour quitter{Colors.RESET}

{Colors.BOLD}════════════════════════════════════════════════════════════════════════════════{Colors.RESET}
"""
    print(banner)

def main():
    print("🎮 Initialisation de pygame...")
    pygame.init()
    pygame.joystick.init()
    
    # Vérifier si une manette est connectée
    if pygame.joystick.get_count() == 0:
        print(f"{Colors.RED}{Colors.BOLD}❌ ERREUR: Aucune manette détectée !{Colors.RESET}")
        print("Veuillez connecter une manette et relancer le script.")
        pygame.quit()
        sys.exit(1)
    
    # Initialiser la première manette
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"{Colors.GREEN}✅ Manette détectée: '{joystick.get_name()}'{Colors.RESET}")
    print(f"{Colors.CYAN}   - Nombre d'axes: {joystick.get_numaxes()}{Colors.RESET}")
    print(f"{Colors.CYAN}   - Nombre de boutons: {joystick.get_numbuttons()}{Colors.RESET}")
    print(f"{Colors.CYAN}   - Nombre de chapeaux (D-pad): {joystick.get_numhats()}{Colors.RESET}")
    
    time.sleep(2)  # Pause pour lire les infos
    print_banner()
    
    # Variables pour l'affichage
    button_states = {}
    axis_values = {}
    hat_values = {}
    
    try:
        clock = pygame.time.Clock()
        running = True
        
        while running:
            for event in pygame.event.get():
                current_time = time.strftime("%H:%M:%S")
                
                if event.type == pygame.JOYBUTTONDOWN:
                    button_states[event.button] = True
                    print(f"{Colors.GREEN}{Colors.BOLD}[{current_time}] BOUTON PRESSÉ: {event.button}{Colors.RESET}")
                
                elif event.type == pygame.JOYBUTTONUP:
                    button_states[event.button] = False
                    print(f"{Colors.RED}[{current_time}] BOUTON RELÂCHÉ: {event.button}{Colors.RESET}")
                
                elif event.type == pygame.JOYAXISMOTION:
                    # Ne pas afficher si la valeur est très proche de 0 (zone morte)
                    if abs(event.value) > 0.1:
                        axis_values[event.axis] = event.value
                        print(f"{Colors.CYAN}[{current_time}] AXE {event.axis}: {event.value:.3f}{Colors.RESET}")
                    else:
                        # Effacer la valeur si elle retourne à 0
                        if event.axis in axis_values:
                            del axis_values[event.axis]
                
                elif event.type == pygame.JOYHATMOTION:
                    hat_values[event.hat] = event.value
                    print(f"{Colors.MAGENTA}[{current_time}] CHAPEAU {event.hat}: {event.value}{Colors.RESET}")
                
                elif event.type == pygame.JOYDEVICEREMOVED:
                    print(f"{Colors.RED}{Colors.BOLD}❌ MANETTE DÉCONNECTÉE !{Colors.RESET}")
                    running = False
                    break
            
            # Affichage de l'état actuel (en bas de l'écran)
            print(f"\r{Colors.BOLD}État actuel:{Colors.RESET}", end="")
            
            # Boutons pressés
            active_buttons = [str(btn) for btn, state in button_states.items() if state]
            if active_buttons:
                print(f" {Colors.GREEN}Boutons: {','.join(active_buttons)}{Colors.RESET}", end="")
            
            # Axes actifs
            if axis_values:
                axis_str = [f"{axis}:{value:.2f}" for axis, value in axis_values.items()]
                print(f" {Colors.CYAN}Axes: {','.join(axis_str)}{Colors.RESET}", end="")
            
            # Chapeaux actifs
            if hat_values:
                hat_str = [f"{hat}:{value}" for hat, value in hat_values.items()]
                print(f" {Colors.MAGENTA}Chapeaux: {','.join(hat_str)}{Colors.RESET}", end="")
            
            print(" " * 20, end="")  # Effacer le reste de la ligne
            
            clock.tick(60)  # 60 FPS pour une réponse fluide
    
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}{Colors.BOLD}🛑 Arrêt demandé par l'utilisateur{Colors.RESET}")
    
    finally:
        pygame.quit()
        print(f"{Colors.CYAN}🎮 Pygame fermé. Au revoir !{Colors.RESET}")

if __name__ == "__main__":
    main()
