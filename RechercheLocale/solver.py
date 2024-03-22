from uflp import UFLP
from typing import List, Tuple
import random
""" 
    Binome 1 : Taylor Jacob (2117518)
    Binome 2 : Bédard-Côté Laurie (2086165)
    Description succinte de l'implementation :
    ...
"""

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
    mainStations = [] # main stations state to return
    satelliteStations = [] # satellite stations links to return

    initialOpenedStations, initialSatelliteStationAssociation = generateInitialSolution(problem) 
    mainStations = initialOpenedStations
    satelliteStations = initialSatelliteStationAssociation

    maxReset = 150
    resetCounter = 0
    bestSolutionFound = (mainStations, satelliteStations)
    maxBestSolutionFoundRepetitions = 5
    bestSolutionFoundRepetitions = 0

    while (resetCounter < maxReset) and (bestSolutionFoundRepetitions < maxBestSolutionFoundRepetitions):
        currentCost = problem.calculate_cost(mainStations, satelliteStations)
        
        neighbors = findNeighboringSolutions(problem, mainStations)
        
        validNeighbors = validateNeighbors(problem, neighbors, currentCost) # remove neighbors where the cost is higher than the current solution
        
        if len(validNeighbors) == 0:
            resetCounter += 1
            print("No valid neighbors found, resetting... ", resetCounter)
            print(currentCost)

            if (bestSolutionFound[0] == mainStations) and (bestSolutionFound[1] == satelliteStations):
                bestSolutionFoundRepetitions += 1
                print("Best solution found repeated ", bestSolutionFoundRepetitions, " times")
            
            elif (currentCost < problem.calculate_cost(bestSolutionFound[0], bestSolutionFound[1])):
                bestSolutionFound = (mainStations, satelliteStations)
                bestSolutionFoundRepetitions = 0
            
            mainStations, satelliteStations = generateRandomSolution(problem)
            continue

        mainStations, satelliteStations = findBestSolution(problem, validNeighbors) # find the best neighbor
        
    return bestSolutionFound

def generateRandomSolution(problem: UFLP) -> Tuple[List[int], List[int]]:
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
    indexForMinCost = 0
    main_stations_opened = [0] * problem.n_main_station

    for i in range(problem.n_main_station):
        if (problem.get_opening_cost(i) < problem.get_opening_cost(indexForMinCost)):
            indexForMinCost = i
    
    main_stations_opened[indexForMinCost] = 1
    satellite_station_association = [indexForMinCost] * problem.n_satellite_station
    
    return main_stations_opened, satellite_station_association

def validateNeighbors(problem: UFLP, neighbors: List[Tuple[List[int], List[int]]], currentCost: float) -> List[Tuple[List[int], List[int]]]:
    validNeighbors = []

    for mainStations, satelliteStations in neighbors:
        if problem.calculate_cost(mainStations, satelliteStations) < currentCost:
            validNeighbors.append((mainStations, satelliteStations))
    
    return validNeighbors

def findNeighboringSolutions(problem: UFLP, mainStations: List[int]) -> List[Tuple[List[int], List[int]]]:
    neighbors = []
    
    for i in range(len(mainStations)):
        # Toggle the station
        mainStations[i] = 1 - mainStations[i]

        # Add the modified array to neighbors if not all stations are closed
        if sum(mainStations) > 0:
            neighbors.append(mainStations.copy())

        # Toggle back to the original state
        mainStations[i] = 1 - mainStations[i]

    result = []

    for i in range(len(neighbors)):
        newSatelliteStations = []
        for j in range(problem.n_satellite_station):
            nearestMainStationDistance = float('inf')  # Initialize with a large value
            nearestMainStationIndex = -1  # Initialize with an invalid index
            
            for k in range(len(neighbors[i])):
                if neighbors[i][k] == 1:
                    cost = problem.get_association_cost(k, j)
                    if cost < nearestMainStationDistance:
                        nearestMainStationDistance = cost
                        nearestMainStationIndex = k
            
            newSatelliteStations.append(nearestMainStationIndex)
        
        result.append((neighbors[i], newSatelliteStations.copy()))
    
    return result

def findBestSolution(problem: UFLP, validNeighbors: List[Tuple[List[int], List[int]]]) -> Tuple[List[int], List[int]]:
    bestCost = float('inf')
    bestMainStations = []
    bestSatelliteStations = []

    for mainStations, satelliteStations in validNeighbors:
        cost = problem.calculate_cost(mainStations, satelliteStations)
        if cost < bestCost:
            # print("New best cost : ", cost)
            bestCost = cost
            bestMainStations = mainStations
            bestSatelliteStations = satelliteStations

    return bestMainStations, bestSatelliteStations