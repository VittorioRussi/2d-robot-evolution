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
of = ea.m.dsToNpnds(npnds = ds.num.mlp())
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
second = ea.m.steppedNds(of = ea.m.dsToNpnds(npnds = ds.num.mlp()); stepT = 0.2)
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

    String BOB = "../../Documents/Experiments/sumo-BO-vs-box.txt/agents";
    String BBB = "../../Documents/Experiments/sumo-BB-vs-box.txt/agents";
    String BOSP = "../../Documents/Experiments/sumo-BO-self-play.txt/agents";
//    String BBSP = "../../Documents/Experiments/sumo-BB-self-play.txt/agents";
//    String BOBIGA = "../../Documents/Experiments/sumo-BO-biGa.txt/agents";
//    String BBBIGA = "../../Documents/Experiments/sumo-BB-biGa.txt/agents";
//    String BBBIME = "../../Documents/Experiments/sumo-BB-biMe.txt/agents";


    String resultsFilePath = "../../Documents/Experiments/Results/results.csv";
    String detailedResultsCsvFilePath = "../../Documents/Experiments/Results/detailedResults.csv";

    try {
//      List<Supplier<CentralizedNumGridVSR>> robots1 = loadRobotsFromDirectory(BOB, boMapper);
//      List<Supplier<DistributedNumGridVSR>> robots2 = loadRobotsFromDirectory(BBB, bbMapper);
      List<Supplier<CentralizedNumGridVSR>> robots3 = loadRobotsFromDirectory(BOSP, boMapper);
//      List<Supplier<DistributedNumGridVSR>> robots4 = loadRobotsFromDirectory(BBSP, bbMapper);
//      List<Supplier<CentralizedNumGridVSR>> robots5 = loadRobotsFromDirectory(BOBIGA, boMapper);
//      List<Supplier<DistributedNumGridVSR>> robots6 = loadRobotsFromDirectory(BBBIGA, bbMapper);
//      List<Supplier<DistributedNumGridVSR>> robots7 = loadRobotsFromDirectory(BBBIME, bbMapper);


      Supplier<Engine> engineSupplier =
          () -> ServiceLoader.load(Engine.class).findFirst().orElseThrow();
      @SuppressWarnings("unchecked")
      Drawer drawer = ((Function<String, Drawer>)
              nb.build(
                  "sim.drawer(framer = sim.staticFramer(minX = 5.0; maxX = 40.0; minY = 10.0; maxY = 25.0); actions = true)"))
          .apply("test");

      List<String> teamNames = List.of("BOB", "BBB", "BOSP", "BBSP");
      List<List<Supplier<? extends EmbodiedAgent>>> allRobots = List.of(
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots1,
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots2,
          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots3
//              ,
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots4,
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots5,
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots6,
//          (List<Supplier<? extends EmbodiedAgent>>) (List<?>) robots7
              );

      String[][] resultsMatrix = new String[allRobots.size()][allRobots.size()];
      for (int i = 0; i < allRobots.size(); i++) {
        for (int j = 0; j < allRobots.size(); j++) {
          resultsMatrix[i][j] = "0;0.0";
        }
      }

      List<String[]> detailedResults = new ArrayList<>();
      detailedResults.add(new String[] {
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

            for (int k = 0; k < team1.size(); k++) {
              Supplier<? extends EmbodiedAgent> robotSupplier1 = team1.get(k);
              for (int p = 0; p < team2.size(); p++) {
                Supplier<? extends EmbodiedAgent> robotSupplier2 = team2.get(p);

                System.out.printf("Agent%d vs Agent%d:%n", k, k);

                Runnable task = taskOn(
                    nb,
                    engineSupplier,
//                    snapshot -> {},
                    new RealtimeViewer(30, drawer),
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
                    detailedResultsCsvFilePath,
                    k,
                    p);
                task.run();
              }
            }
          }
        }
      }

      saveResultsAsCsv(resultsFilePath, teamNames, resultsMatrix);
      saveDetailedResults(detailedResultsCsvFilePath, detailedResults);
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

  public static void saveResultsAsCsv(String filePath, List<String> teamNames, String[][] resultsMatrix)
      throws IOException {
    try (FileWriter writer = new FileWriter(filePath)) {

      writer.append("Home Team\\Away Team,");

      for (String teamName : teamNames) {
        writer.append(teamName).append(",");
      }
      writer.append("\n");

      for (int i = 0; i < teamNames.size(); i++) {

        writer.append(teamNames.get(i)).append(",");
        for (int j = 0; j < teamNames.size(); j++) {
          if (i != j) {
            writer.append(resultsMatrix[i][j]).append(",");
          } else {
            writer.append("N/A,");
          }
        }
        writer.append("\n");
      }
    }
  }

  public static void saveDetailedResults(String filePath, List<String[]> detailedResults) throws IOException {
    try (FileWriter writer = new FileWriter(filePath)) {
      for (String[] result : detailedResults) {
        writer.append(String.join(",", result)).append("\n");
      }
    }
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
      String detailedResultsCsvFilePath,
      int agentNumber1,
      int agentNumber2) {
    return () -> {
      try {
        Sumo sumo = new Sumo(5);
//        Engine engine = engineSupplier.get();
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

        detailedResults.add(new String[] {
          teamName1,
          String.valueOf(agentNumber1),
          teamName2,
          String.valueOf(agentNumber2),
          homeWins ? "T" : "F",
          String.format("%.1f", homeScore),
          String.format("%.1f", awayScore),
          String.format("%.3f", duration)
        });

        String currentResults = resultsMatrix[teamIndex1][teamIndex2];
        currentResults = currentResults.replace(",", ".");

        if (currentResults == null || currentResults.isEmpty() || currentResults.equals("0;0.0")) {
          currentResults = "V: 0; S = 0.0";
        }

        String[] parts = currentResults.split("; ");
        if (parts.length != 2) {
          throw new IllegalArgumentException("Invalid format of the string: " + currentResults);
        }

        int homeVictories = Integer.parseInt(parts[0].split(": ")[1].trim());
        double homeScoreTotal = Double.parseDouble(parts[1].split("= ")[1].trim());

        if (isHome) {
          homeVictories++;
          homeScoreTotal += score1vs2;
        }

        double averageScore = homeScoreTotal / homeVictories;
        resultsMatrix[teamIndex1][teamIndex2] = String.format("V: %d; S = %.1f", homeVictories, averageScore);
      } catch (Exception e) {
        e.printStackTrace();
      }
    };
  }
}
