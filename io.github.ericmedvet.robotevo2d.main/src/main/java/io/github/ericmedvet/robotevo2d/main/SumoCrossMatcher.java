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
import io.github.ericmedvet.jnb.datastructure.DoubleRange;
import io.github.ericmedvet.jnb.datastructure.NumericalParametrized;
import io.github.ericmedvet.jsdynsym.core.composed.Composed;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.NumMultiBrained;
import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.CentralizedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.geometry.Terrain;
import io.github.ericmedvet.mrsim2d.core.tasks.sumo.Sumo;
import io.github.ericmedvet.mrsim2d.core.tasks.trainingsumo.TrainingSumo;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.Base64;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;
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

      System.out.println(robots1.getFirst().get());

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
  private static Runnable taskOn(
          NamedBuilder<?> nb, Supplier<Engine> engineSupplier, Consumer<Snapshot> consumer, String terrain, Supplier<EmbodiedAgent> agentSupplier1, Supplier<EmbodiedAgent> agentSupplier2) {
    // prepare task
    Sumo sumo = new Sumo(30, (Terrain) nb.build(terrain));

    return () -> sumo.run(agentSupplier1, agentSupplier2, engineSupplier.get(), consumer);
  }
}
