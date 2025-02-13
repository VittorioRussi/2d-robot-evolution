/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2018 - 2025 Eric Medvet
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package io.github.ericmedvet.robotevo2d.main;

import static io.github.ericmedvet.mrsim2d.buildable.builders.OutcomeFunctions.scoreSumoAgent1vs2;
import static io.github.ericmedvet.mrsim2d.buildable.builders.OutcomeFunctions.scoreSumoAgent2vs1;

import io.github.ericmedvet.jgea.core.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.CentralizedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.sumo.Sumo;
import io.github.ericmedvet.mrsim2d.core.tasks.sumo.SumoAgentsOutcome;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import io.github.ericmedvet.mrsim2d.viewer.RealtimeViewer;

import java.io.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.Base64;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class SumoCrossMatcher {
    private static final String BO_MAPPER =
            """
                    er.m.ndsToFixedBodyCentralizedVSR(
                    body = s.a.vsr.gridBody(
                    sensorizingFunction = s.a.vsr.sf.directional(
                    headSensors = [s.s.sin(f = 0); s.s.d(a = -15; r = 5)];
                    nSensors = [s.s.ar(); s.s.v(a = 0); s.s.v(a = 90)];
                    sSensors = [s.s.d(a = -90)]
                    );
                    shape = s.a.vsr.s.biped(w = 4; h = 3)
                    );
                    of = ea.m.noisedNds(of = ea.m.steppedNds(of = ea.m.dsToNpnds(npnds = ds.num.mlp()); stepT = 0.2); inputSigma = 0.05)
                    )
                    """;
    private static final String BB_MAPPER =
            """
                    er.m.bodyBrainHomoDistributedVSR(
                    w = 8;
                    h = 8;
                    sensors = [s.s.ar(); s.s.v(a = 0); s.s.v(a = 90)];
                    of = ea.m.pair(
                    of = ea.m.dsSplit();
                    first = ea.m.dsToFixedGrid(negItem = s.a.vsr.voxel(type = none); posItem = s.a.vsr.voxel(type = soft));
                    second = ea.m.noisedNds(of = ea.m.steppedNds(of = ea.m.dsToNpnds(npnds = ds.num.mlp()); stepT = 0.2); inputSigma = 0.05)
                    )
                    )
                    """;

    public static void main(String[] args) {
        NamedBuilder<?> nb = NamedBuilder.fromDiscovery();
        @SuppressWarnings("unchecked")
        InvertibleMapper<List<Double>, Supplier<CentralizedNumGridVSR>> boMapper =
                (InvertibleMapper<List<Double>, Supplier<CentralizedNumGridVSR>>) nb.build(BO_MAPPER);
        @SuppressWarnings("unchecked")
        InvertibleMapper<List<Double>, Supplier<DistributedNumGridVSR>> bbMapper =
                (InvertibleMapper<List<Double>, Supplier<DistributedNumGridVSR>>) nb.build(BB_MAPPER);

        String BOB = "../Experiments/sumo-BO-vs-box.txt/agents";
        String BBB = "../Experiments/sumo-BB-vs-box.txt/agents";
        String BOSP = "../Experiments/sumo-BO-self-play.txt/agents";
        String BBSP = "../Experiments/sumo-BB-self-play.txt/agents";
        String BOBIGA = "../Experiments/sumo-BO-biGa.txt/agents";
        String BBBIGA = "../Experiments/sumo-BB-biGa.txt/agents";
        String BBBIME = "../Experiments/sumo-BB-biMe.txt/agents";

        String resultsFilePath = "../Experiments/Results/results.csv";
        String detailedResultsCsvFilePath = "../Experiments/Results/detailedResults.csv";

        try {
            List<Supplier<CentralizedNumGridVSR>> robots1 = loadRobotsFromDirectory(BOB, boMapper);
//            List<Supplier<DistributedNumGridVSR>> robots2 = loadRobotsFromDirectory(BBB, bbMapper);
            List<Supplier<CentralizedNumGridVSR>> robots3 = loadRobotsFromDirectory(BOSP, boMapper);
            List<Supplier<DistributedNumGridVSR>> robots4 = loadRobotsFromDirectory(BBSP, bbMapper);
            List<Supplier<CentralizedNumGridVSR>> robots5 = loadRobotsFromDirectory(BOBIGA, boMapper);
            List<Supplier<DistributedNumGridVSR>> robots6 = loadRobotsFromDirectory(BBBIGA, bbMapper);
            List<Supplier<DistributedNumGridVSR>> robots7 = loadRobotsFromDirectory(BBBIME, bbMapper);

            Supplier<Engine> engineSupplier =
                    () -> ServiceLoader.load(Engine.class).findFirst().orElseThrow();
            @SuppressWarnings("unchecked")
            Drawer drawer = ((Function<String, Drawer>)
                    nb.build(
                            "sim.drawer(framer = sim.staticFramer(minX = 5.0; maxX = 40.0; minY = 10.0; maxY = 25.0); actions = true)"))
                    .apply("test");

            List<String> teamNames = List.of("BOB", "BOSP", "BBSP", "BOBIGA", "BBBIGA", "BBBIME");
            List<List<Supplier<? extends EmbodiedAgent>>> allRobots = List.of(
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots1,
//                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots2,
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots3,
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots4,
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots5,
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots6,
                    (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots7
            );

            String[][] resultsMatrix = new String[allRobots.size()][allRobots.size()];
            for (int i = 0; i < allRobots.size(); i++) {
                for (int j = 0; j < allRobots.size(); j++) {
                    resultsMatrix[i][j] = "0;0.0";
                }
            }

            List<String[]> detailedResults = new ArrayList<>();
            detailedResults.add(new String[]{
                    "HomeTeam",
                    "HomeAgent",
                    "AwayTeam",
                    "AwayAgent",
                    "VictoryHome",
                    "ScoreHomeAgent",
                    "ScoreAwayAgent",
                    "TimeFight"
            });

            for (int i = 0; i < allRobots.size(); i++) {
                for (int j = 0; j < allRobots.size(); j++) {
                    if (i != j) {
                        List<Supplier<? extends EmbodiedAgent>> team1 = allRobots.get(i);
                        List<Supplier<? extends EmbodiedAgent>> team2 = allRobots.get(j);

                        String teamName1 = teamNames.get(i);
                        String teamName2 = teamNames.get(j);

                        System.out.printf("Start of the fight Team%d vs Team%d:%n", i + 1, j + 1);

                        for (int k = 0; k < 3; k++) {
                            Supplier<? extends EmbodiedAgent> robotSupplier1 = team1.get(k);
                            for (int p = 0; p < 3; p++) {
                                Supplier<? extends EmbodiedAgent> robotSupplier2 = team2.get(p);

                                System.out.printf("Agent%d vs Agent%d:%n", k, p);

                                Runnable task = taskOn(
                                        nb,
                                        engineSupplier,
                                        snapshot -> {},
                                        "s.t.sumoArena()",
                                        robotSupplier1,
                                        robotSupplier2,
                                        new ArrayList<>(),
                                        teamName1,
                                        teamName2,
                                        resultsMatrix,
                                        detailedResults,
                                        i,
                                        j,
                                        true,
                                        k,
                                        p);
                                task.run();
                            }
                        }
                    }
                }
            }

            saveAllResults(resultsFilePath, detailedResultsCsvFilePath, teamNames, resultsMatrix, detailedResults);
            System.exit(0);
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static List<Double> fromBase64(String content) throws IOException, ClassNotFoundException {
        try (ByteArrayInputStream bais =
                     new ByteArrayInputStream(Base64.getDecoder().decode(content));
             ObjectInputStream ois = new ObjectInputStream(bais)) {
            return (List<Double>) ois.readObject();
        } catch (Throwable t) {
            throw new IOException(t);
        }
    }

    public static void saveAllResults(String resultsFilePath, String detailedResultsFilePath, List<String> teamNames, String[][] resultsMatrix, List<String[]> detailedResults) throws IOException {
        String teamWithMostHomeWins = getTeamWithMostHomeWins(detailedResults);
        String teamWithMostAwayWins = getTeamWithMostAwayWins(detailedResults);
        String teamWithBestAverageScore = getTeamWithBestAverageScore(detailedResults, teamNames);
        Map<String, Double> averageBattleTime = getAverageBattleTime(detailedResults);

        try (FileWriter writer = new FileWriter(resultsFilePath)) {
            writer.append("Home Team\\Away Team,");

            for (String teamName : teamNames) {
                writer.append(teamName).append(",");
            }
            writer.append("\n");

            for (int i = 0; i < teamNames.size(); i++) {
                writer.append(teamNames.get(i)).append(",");
                for (int j = 0; j < teamNames.size(); j++) {
                    writer.append(resultsMatrix[i][j]).append(",");
                }
                writer.append("\n");
            }

            writer.append("\nSummary Information\n");
            writer.append("Team with most home wins: ").append(teamWithMostHomeWins).append("\n");
            writer.append("Team with most away wins: ").append(teamWithMostAwayWins).append("\n");
            writer.append("Team with best average score: ").append(teamWithBestAverageScore).append("\n");
        }

        try (FileWriter writer = new FileWriter(detailedResultsFilePath)) {
            for (String[] result : detailedResults) {
                writer.append(String.join(",", result)).append("\n");
            }

            writer.append("\nSummary Information\n");
            writer.append("Team with most home wins: ").append(teamWithMostHomeWins).append("\n");
            writer.append("Team with most away wins: ").append(teamWithMostAwayWins).append("\n");
            writer.append("Team with best average score: ").append(teamWithBestAverageScore).append("\n");
            for (Map.Entry<String, Double> entry : averageBattleTime.entrySet()) {
                writer.append("Average battle time for ").append(entry.getKey()).append(": ").append(String.format("%.3f", entry.getValue())).append("\n");
            }
        }
    }

    public static Map<String, Double> getAverageBattleTime(List<String[]> detailedResults) {
        Map<String, List<Double>> battleTimes = new HashMap<>();
        for (int i = 1; i < detailedResults.size(); i++) {
            String[] result = detailedResults.get(i);
            if (result.length > 7 && "T".equals(result[4])) {
                battleTimes.computeIfAbsent(result[0], k -> new ArrayList<>()).add(Double.parseDouble(result[7].replace(",", ".")));
            }
        }
        Map<String, Double> averageBattleTimes = new HashMap<>();
        for (Map.Entry<String, List<Double>> entry : battleTimes.entrySet()) {
            averageBattleTimes.put(entry.getKey(), entry.getValue().stream().mapToDouble(Double::doubleValue).average().orElse(0));
        }
        return averageBattleTimes;
    }

    public static String getTeamWithMostHomeWins(List<String[]> detailedResults) {
        Map<String, Integer> homeWins = new HashMap<>();
        for (int i = 1; i < detailedResults.size(); i++) {
            String[] result = detailedResults.get(i);
            if (result.length > 4 && "T".equals(result[4])) {
                homeWins.put(result[0], homeWins.getOrDefault(result[0], 0) + 1);
            }
        }
        return homeWins.entrySet().stream().max(Map.Entry.comparingByValue()).map(Map.Entry::getKey).orElse(null);
    }

    public static String getTeamWithMostAwayWins(List<String[]> detailedResults) {
        Map<String, Integer> awayWins = new HashMap<>();
        for (int i = 1; i < detailedResults.size(); i++) {
            String[] result = detailedResults.get(i);
            if (result.length > 4 && "F".equals(result[4])) {
                awayWins.put(result[2], awayWins.getOrDefault(result[2], 0) + 1);
            }
        }
        return awayWins.entrySet().stream().max(Map.Entry.comparingByValue()).map(Map.Entry::getKey).orElse(null);
    }

    public static String getTeamWithBestAverageScore(List<String[]> detailedResults, List<String> teamNames) {
        Map<String, List<Double>> scores = new HashMap<>();
        for (int i = 1; i < detailedResults.size(); i++) {
            String[] result = detailedResults.get(i);
            if (result.length > 5) {
                try {
                    scores.computeIfAbsent(result[0], k -> new ArrayList<>()).add(Double.parseDouble(result[5].replace(",", ".")));
                    scores.computeIfAbsent(result[2], k -> new ArrayList<>()).add(Double.parseDouble(result[6].replace(",", ".")));
                } catch (NumberFormatException e) {
                    System.err.println("Errore di formattazione del numero: " + e.getMessage());
                }
            }
        }
        return scores.entrySet().stream()
                .max(Comparator.comparingDouble(e -> e.getValue().stream().mapToDouble(Double::doubleValue).average().orElse(0)))
                .map(Map.Entry::getKey).orElse(null);
    }

    public static List<Double> loadGenotypeFromFile(String filePath) throws IOException, ClassNotFoundException {
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line = br.readLine();
            if (line != null) {
                return fromBase64(line);
            } else {
                throw new IOException("Empty file or invalid format");
            }
        }
    }

    public static <T> List<Supplier<T>> loadRobotsFromDirectory(
            String dirPath, InvertibleMapper<List<Double>, Supplier<T>> mapper)
            throws IOException, ClassNotFoundException {
        List<Supplier<T>> robots = new ArrayList<>();
        try (Stream<Path> paths = Files.list(Paths.get(dirPath))) {
            robots = paths.filter(Files::isRegularFile)
                    .map(filePath -> {
                        try {
                            List<Double> genotype = loadGenotypeFromFile(filePath.toString());
                            return mapper.mapperFor(null).apply(genotype);
                        } catch (IOException | ClassNotFoundException e) {
                            e.printStackTrace();
                            return null;
                        }
                    })
                    .filter(Objects::nonNull)
                    .collect(Collectors.toList());
        }
        return robots;
    }

    public static <T1 extends EmbodiedAgent, T2 extends EmbodiedAgent> Runnable taskOn(
            NamedBuilder<?> nb,
            Supplier<Engine> engineSupplier,
            Consumer<Snapshot> consumer,
            String terrain,
            Supplier<T1> agentSupplier1,
            Supplier<T2> agentSupplier2,
            List<Double> scores1vs2,
            String teamName1,
            String teamName2,
            String[][] resultsMatrix,
            List<String[]> detailedResults,
            int teamIndex1,
            int teamIndex2,
            boolean isHome,
            int agentNumber1,
            int agentNumber2) {
        return () -> {
            try {
                Sumo sumo = new Sumo(30);
                SumoAgentsOutcome outcome = sumo.run(
                        (Supplier<EmbodiedAgent>) agentSupplier1,
                        (Supplier<EmbodiedAgent>) agentSupplier2,
                        engineSupplier.get(),
                        consumer);
                double duration = outcome.duration();

                double score1vs2 =
                        scoreSumoAgent1vs2(0.0, Function.identity(), "%.1f").apply(outcome);
                double score2vs1 =
                        scoreSumoAgent2vs1(0.0, Function.identity(), "%.1f").apply(outcome);
                scores1vs2.add(score1vs2);

                boolean homeWins = score1vs2 > score2vs1;

                double homeScore = score1vs2;
                double awayScore = score2vs1;

                detailedResults.add(new String[]{
                        teamName1,
                        String.valueOf(agentNumber1),
                        teamName2,
                        String.valueOf(agentNumber2),
                        homeWins ? "T" : "F",
                        String.format("%.1f", homeScore),
                        String.format("%.1f", awayScore),
                        String.format("%.3f", duration)
                });

                resultsMatrix[teamIndex1][teamIndex2] = homeWins ? "T" : "F";
            } catch (Exception e) {
                e.printStackTrace();
            }
        };
    }
}