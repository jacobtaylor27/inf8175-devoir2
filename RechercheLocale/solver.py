from uflp import UFLP
from typing import List, Tuple
import random
import time
""" 
    Binome 1 : Taylor Jacob (2117518)
    Binome 2 : Bédard-Côté Laurie (2086165)
    Méthode de résolution : Recherche locale avec l'algorithme de Hill Climbing
    Description succinte de l'implementation :
        - On commence par générer une solution initiale en ouvrant seulement la station avec le coût d'ouverture minimum et en associant toutes les stations satellites à cette station
        - On génère des solutions voisines en ouvrant une station fermée à la fois
        - On valide les solutions voisines en ne gardant que celles dont le coût est inférieur à la solution actuelle
        - On choisit la meilleure solution parmi les solutions valides
        - Si on atteint un minimum local, on génère une nouvelle solution aléatoire
        - On répète les étapes précédentes jusqu'à ce qu'on atteigne un des critères d'arrêt (temps écoulé, nombre de resets, convergence)
        - On retourne la meilleure solution trouvée
"""

# These constants are kind of arbitrary, but they seem to work well for the given problem instances
MAX_RESET = 150 # Maximum number of resets for the local search (resets are done when the search reaches a local minimum)
MAX_CONVERGENCE = 10 # Maximum number of times a solution can be found until we accept the best solution found as the final solution
MAX_SECONDS = 100 # Maximum number of seconds for the local search (little less then 2 minutes)

def solve(problem: UFLP) -> Tuple[List[int], List[int]]:
    """
    Votre implementation, doit resoudre le probleme via recherche locale.

    Args:
        problem (UFLP): L'instance du probleme à résoudre

    Returns:
        Tuple[List[int], List[int]]: 
        La premiere valeur est une liste représentant les stations principales ouvertes au format [0, 1, 0] qui indique que seule la station 1 est ouverte
        La seconde valeur est une liste représentant les associations des stations satellites au format [1 , 4] qui indique que la premiere station est associée à la station pricipale d'indice 1 et la deuxieme à celle d'indice 4
    """

    # Start the timer
    startTime = time.time()

    # Generate the initial solution
    mainStations, satelliteStations = generateInitialSolution(problem)
    
    resetCounter = 0
    convergenceCounter = 0 # Counter to check if the best solution found is repeated multiple times (converging)
    bestSolutionFound = (mainStations, satelliteStations) # Best solution found so far (initialize with the initial solution)

    while not shouldStop(convergenceCounter, resetCounter, startTime):
        currentCost = problem.calculate_cost(mainStations, satelliteStations) # Calculate the cost of the current solution
        neighbors = findNeighboringSolutions(problem, mainStations) # find the neighbors of the current solution
        validNeighbors = validateNeighbors(problem, neighbors, currentCost) # remove neighbors where the cost is higher than the current solution
        
        if len(validNeighbors) == 0: # this means that the current solution is a local minimum
            resetCounter += 1
            # print(f'RESET COUNTER -> [{resetCounter}]')

            # Check if the best solution found is repeated multiple times (converging)
            if (bestSolutionFound[0] == mainStations) and (bestSolutionFound[1] == satelliteStations):
                convergenceCounter += 1
                # print(f'CONVERGENCE COUNTER -> [{convergenceCounter}]')

            # Update the best solution found if the current solution is better            
            elif (currentCost < problem.calculate_cost(bestSolutionFound[0], bestSolutionFound[1])):
                bestSolutionFound = (mainStations, satelliteStations)
                convergenceCounter = 0 # Reset the convergence counter because we found a better solution
                # print(f'NEW COST -> [{round(currentCost)}]')
            
            # Generate a new random solution
            mainStations, satelliteStations = generateRandomSolution(problem)
            continue
        
        mainStations, satelliteStations = findBestSolution(problem, validNeighbors) # find the best neighbor
        
    return bestSolutionFound

def shouldStop(convergenceCounter: int, resetCounter: int, startTime: float) -> bool:
    """
    Check if the search should stop. The conditions are : 
    - The best solution found is repeated multiple times (converging)
    - The number of resets exceeds the maximum number of resets
    - The time elapsed exceeds the maximum time allowed

    Args:
        convergenceCounter (int): Counter to check if the best solution found is repeated multiple times (converging)
        resetCounter (int): Counter to check the number of resets
        timeElapsed (float): Time elapsed since the start of the search

    Returns:
        bool: True if the search should stop, False otherwise
    """

    currentTime = round(time.time() - startTime, 2)
    return (convergenceCounter >= MAX_CONVERGENCE) or (resetCounter >= MAX_RESET) or (currentTime >= MAX_SECONDS)

def generateRandomSolution(problem: UFLP) -> Tuple[List[int], List[int]]:
    """
    Retourne une solution aléatoire au probleme : à battre pour avoir la moyenne.
    Cette implémentation a été reprise de random_solver.py

    Args:
        problem (UFLP): L'instance du probleme à résoudre

    Returns:
        Tuple[List[int], List[int]]: 
        La premiere valeur est une liste représentant les stations principales ouvertes au format [0, 1, 0] qui indique que seule la station 1 est ouverte
        La seconde valeur est une liste représentant les associations des stations satellites au format [1 , 4] qui indique que la premiere station est associée à la station pricipale d'indice 1 et la deuxieme à celle d'indice 4
    """

    # Ouverture aléatoire des stations principales
    main_stations_opened = [random.choice([0,1]) for _ in range(problem.n_main_station)]

    # Si, par hasard, rien n'est ouvert, on ouvre une station aléatoirement
    if sum(main_stations_opened) == 0:
        main_stations_opened[random.choice(range(problem.n_main_station))] = 1

    # Association aléatoire des stations satellites aux stations principales ouvertes
    indices = [i for i in range(len(main_stations_opened)) if main_stations_opened[i] == 1]
    satellite_station_association = [random.choice(indices) for _ in range(problem.n_satellite_station)]

    return main_stations_opened, satellite_station_association

def generateInitialSolution(problem: UFLP) -> Tuple[List[int], List[int]]:
    """
    Generate the initial solution by opening only the station with the minimum opening cost and associating all satellite stations to it

    Args:
        problem (UFLP): The problem instance

    Returns:
        Tuple[List[int], List[int]]: The initial solution
    """

    minCostIndex = 0
    main_stations_opened = [0] * problem.n_main_station # All stations are closed initially

    # Find the station with the minimum opening cost
    for i in range(problem.n_main_station):
        if (problem.get_opening_cost(i) < problem.get_opening_cost(minCostIndex)):
            minCostIndex = i
    
    # Open the station with the minimum opening cost
    main_stations_opened[minCostIndex] = 1

    # Associate all satellite stations to the station with the minimum opening cost
    satellite_station_association = [minCostIndex] * problem.n_satellite_station
    
    return main_stations_opened, satellite_station_association

def validateNeighbors(problem: UFLP, neighbors: List[Tuple[List[int], List[int]]], currentCost: float) -> List[Tuple[List[int], List[int]]]:
    """
    Validate the neighbors by removing the ones with a cost higher than the current solution

    Args:
        problem (UFLP): The problem instance
        neighbors (List[Tuple[List[int], List[int]]]): List of neighbors
        currentCost (float): The cost of the current solution

    Returns:
        List[Tuple[List[int], List[int]]]: List of valid neighbors
    """

    validNeighbors = []

    # Check if the cost of the neighbor is less than the current cost
    for mainStations, satelliteStations in neighbors:
        if problem.calculate_cost(mainStations, satelliteStations) < currentCost:
            validNeighbors.append((mainStations, satelliteStations))
    
    return validNeighbors

def findNeighboringSolutions(problem: UFLP, mainStations: List[int]) -> List[Tuple[List[int], List[int]]]:
    """
    Finds the neighboring solutions of the current solution

    Args:
        problem (UFLP): The problem instance
        mainStations (List[int]): The state of the main stations for the current solution

    Returns:
        List[Tuple[List[int], List[int]]]: List of neighboring solutions
    """
    
    neighbors = []
    
    for i in range(len(mainStations)):
        # Toggle the station
        mainStations[i] = 1 - mainStations[i]

        # Add the modified array to neighbors if not all stations are closed
        if sum(mainStations) > 0:
            neighbors.append(mainStations.copy())

        # Toggle back to the original state
        mainStations[i] = 1 - mainStations[i]

    neighboringSolutions = []

    for i in range(len(neighbors)):
        newSatelliteStations = [] # List of satellite stations for the new solution
        
        # Find the nearest main station for each satellite station
        for j in range(problem.n_satellite_station):
            nearestMainStationDistance = float('inf')  # Initialize with a large value
            nearestMainStationIndex = -1  # Initialize with an invalid index
            
            # Find the nearest main station
            for k in range(len(neighbors[i])):
                if neighbors[i][k] == 1:
                    cost = problem.get_association_cost(k, j)

                    # Update the nearest main station if the cost is less than the current nearest main station distance
                    if cost < nearestMainStationDistance:
                        nearestMainStationDistance = cost
                        nearestMainStationIndex = k
            
            # Add the nearest main station index to the satellite stations list
            newSatelliteStations.append(nearestMainStationIndex)
        
        # Add the new solution to the list of neighboring solutions
        neighboringSolutions.append((neighbors[i], newSatelliteStations.copy()))
    
    return neighboringSolutions

def findBestSolution(problem: UFLP, validNeighbors: List[Tuple[List[int], List[int]]]) -> Tuple[List[int], List[int]]:
    """
    Find the best solution among the valid neighbors

    Args:
        problem (UFLP): The problem instance
        validNeighbors (List[Tuple[List[int], List[int]]]): List of valid neighbors

    Returns:
        Tuple[List[int], List[int]]: The best solution
    """
    
    bestCost = float('inf')
    bestMainStations = []
    bestSatelliteStations = []

    for mainStations, satelliteStations in validNeighbors:
        cost = problem.calculate_cost(mainStations, satelliteStations)
        if cost < bestCost:
            bestCost = cost
            bestMainStations = mainStations
            bestSatelliteStations = satelliteStations

    return bestMainStations, bestSatelliteStations