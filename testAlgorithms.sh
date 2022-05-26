#!/bin/bash

cd search

resultFile=../results.txt

rm -f $resultFile
touch $resultFile
mazes=(tinyMaze mediumMaze bigMaze)
cornerMazes=(tinyCorners mediumCorners)
searchAlgorithms=(dfs bfs ucs astar)


for maze in ${mazes[@]}; do
  echo $maze >> $resultFile
  for searchAlgorithm in ${searchAlgorithms[@]}; do
    echo -n $searchAlgorithm" " >> $resultFile
    if [[ $searchAlgorithm = astar ]]; then
      echo -n with manhattanHeuristic" " >> $resultFile
    fi
    if [[ $searchAlgorithm = ucs ]]; then
      ucsSearchagent=StayWestSearchAgent
      echo -n with $ucsSearchagent" " >> $resultFile
      python pacman.py -l $maze -p $ucsSearchagent -q \
      | awk 'FNR == 1 {printf("pathCost: %s ", $7)} FNR == 2 {printf("nodesExpanded: %s\n", $4)}' >> $resultFile
    else
      python pacman.py -l $maze -p SearchAgent -q -a fn=${searchAlgorithm},heuristic=manhattanHeuristic \
      | awk 'FNR == 3 {printf("pathCost: %s ", $7)} FNR == 4 {printf("nodesExpanded: %s\n", $4)}' >> $resultFile
    fi
  done
done

echo CornersProblem >> $resultFile
for maze in ${cornerMazes[@]}; do
  echo $maze >> $resultFile
  for searchAlgorithm in ${searchAlgorithms[@]}; do
    echo -n $searchAlgorithm" " >> $resultFile
    if [[ $searchAlgorithm = astar ]]; then
      echo -n with AStarCornersAgent" " >> $resultFile
      python pacman.py -l $maze -p AStarCornersAgent\
      | awk 'FNR == 1 {printf("pathCost: %s ", $7)} FNR == 2 {printf("nodesExpanded: %s\n", $4)}' >> $resultFile
    else
      python pacman.py -l $maze -p SearchAgent -q -a fn=${searchAlgorithm},prob=CornersProblem \
      | awk 'FNR == 3 {printf("pathCost: %s ", $7)} FNR == 4 {printf("nodesExpanded: %s\n", $4)}' >> $resultFile
    fi
  done
done