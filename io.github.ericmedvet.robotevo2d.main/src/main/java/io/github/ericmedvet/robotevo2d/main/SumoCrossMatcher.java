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

import io.github.ericmedvet.jgea.core.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.CentralizedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.geometry.Terrain;
import io.github.ericmedvet.mrsim2d.core.tasks.sumo.Sumo;
import io.github.ericmedvet.mrsim2d.core.tasks.sumo.SumoAgentsOutcome;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import io.github.ericmedvet.mrsim2d.viewer.RealtimeViewer;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.Base64;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import static io.github.ericmedvet.mrsim2d.buildable.builders.OutcomeFunctions.scoreSumoAgent1vs2;
import static io.github.ericmedvet.mrsim2d.buildable.builders.OutcomeFunctions.scoreSumoAgent2vs1;

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
                    sensors = [s.s.a(); s.s.ar(); s.s.rv(a = 0); s.s.rv(a = 90)];
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
        String BBSP = "../../Documents/Experiments/sumo-BB-self-play.txt/agents";

        try {
            List<Supplier<CentralizedNumGridVSR>> robots1 = loadRobotsFromDirectory(BOB, boMapper);
            List<Supplier<DistributedNumGridVSR>> robots2 = loadRobotsFromDirectory(BBB, bbMapper);
            List<Supplier<CentralizedNumGridVSR>> robots3 = loadRobotsFromDirectory(BOSP, boMapper);
            List<Supplier<DistributedNumGridVSR>> robots4 = loadRobotsFromDirectory(BBSP, bbMapper);

            // prepare engine
            Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class).findFirst().orElseThrow();

            // prepare drawer
            @SuppressWarnings("unchecked")
            Drawer drawer = ((Function<String, Drawer>) nb.build(
                    "sim.drawer(framer = sim.staticFramer(minX = 10.0; maxX = 35.0; minY = 1.0; maxY = 15.0); actions = true)"))
                    .apply("test");

            // Liste per salvare i punteggi
            List<Double> scores1vs2 = new ArrayList<>();
            List<Double> scores2vs1 = new ArrayList<>();

            // Contatori per le vittorie
            int victoriesTeam1 = 0;
            int victoriesTeam2 = 0;

            // Lista di tutte le liste di robot
            List<List<Supplier<? extends EmbodiedAgent>>> allRobots = List.of(
                    (List<Supplier<? extends EmbodiedAgent>>)(List<?>)robots1,
                    (List<Supplier<? extends EmbodiedAgent>>)(List<?>)robots2,
                    (List<Supplier<? extends EmbodiedAgent>>)(List<?>)robots3,
                    (List<Supplier<? extends EmbodiedAgent>>)(List<?>)robots4
            );

            // Itera su tutte le combinazioni di liste di robot
            for (int i = 0; i < allRobots.size(); i++) {
                for (int j = 0; j < allRobots.size(); j++) {
                    if (i != j) {
                        List<Supplier<? extends EmbodiedAgent>> team1 = allRobots.get(i);
                        List<Supplier<? extends EmbodiedAgent>> team2 = allRobots.get(j);

                        // Scontri team1 contro team2
                        for (Supplier<? extends EmbodiedAgent> robotSupplier1 : team1) {
                            for (Supplier<? extends EmbodiedAgent> robotSupplier2 : team2) {
                                Runnable task = taskOn(
                                        nb,
                                        engineSupplier,
                                        new RealtimeViewer(30, drawer),
                                        "s.t.sumoArena()",
                                        robotSupplier1,
                                        robotSupplier2,
                                        scores1vs2,
                                        scores2vs1
                                );
                                task.run();
                            }
                        }

                        // Scontri team2 contro team1 (cambio casa-trasferta)
                        for (Supplier<? extends EmbodiedAgent> robotSupplier2 : team2) {
                            for (Supplier<? extends EmbodiedAgent> robotSupplier1 : team1) {
                                Runnable task = taskOn(
                                        nb,
                                        engineSupplier,
                                        new RealtimeViewer(30, drawer),
                                        "s.t.sumoArena()",
                                        robotSupplier2,
                                        robotSupplier1,
                                        scores2vs1,
                                        scores1vs2
                                );
                                task.run();
                            }
                        }
                    }
                }
            }

            // Calcola il numero di vittorie
            for (int i = 0; i < scores1vs2.size(); i++) {
                if (scores1vs2.get(i) > scores2vs1.get(i)) {
                    victoriesTeam1++;
                } else if (scores2vs1.get(i) > scores1vs2.get(i)) {
                    victoriesTeam2++;
                }
            }

            // Calcola il punteggio medio per ciascuna squadra
            double averageScoreTeam1 = scores1vs2.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
            double averageScoreTeam2 = scores2vs1.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

            // Salva i risultati in un file CSV
            try (FileWriter writer = new FileWriter("results.csv")) {
                writer.append("Team,Victories,Average Score\n");
                writer.append("Team 1,").append(String.valueOf(victoriesTeam1)).append(",").append(String.valueOf(averageScoreTeam1)).append("\n");
                writer.append("Team 2,").append(String.valueOf(victoriesTeam2)).append(",").append(String.valueOf(averageScoreTeam2)).append("\n");
            } catch (IOException e) {
                e.printStackTrace();
            }

            System.out.println("I risultati sono stati salvati in results.csv");

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

    private static <T1 extends EmbodiedAgent, T2 extends EmbodiedAgent> Runnable taskOn(
            NamedBuilder<?> nb,
            Supplier<Engine> engineSupplier,
            Consumer<Snapshot> consumer,
            String terrain,
            Supplier<T1> agentSupplier1,
            Supplier<T2> agentSupplier2,
            List<Double> scores1vs2,
            List<Double> scores2vs1) {
        return () -> {
            // prepare task
            Sumo sumo = new Sumo(30, (Terrain) nb.build(terrain));
            SumoAgentsOutcome outcome = sumo.run(
                    (Supplier<EmbodiedAgent>) agentSupplier1,
                    (Supplier<EmbodiedAgent>) agentSupplier2,
                    engineSupplier.get(),
                    consumer
            );

            // Calcola i punteggi
            double score1vs2 = scoreSumoAgent1vs2(0.0, Function.identity(), "%.1f").apply(outcome);
            double score2vs1 = scoreSumoAgent2vs1(0.0, Function.identity(), "%.1f").apply(outcome);

            // Salva i risultati
            scores1vs2.add(score1vs2);
            scores2vs1.add(score2vs1);
        };
    }
}