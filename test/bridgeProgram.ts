// Drives the compiled C++ algorithm as if it were a TypeScript RobotProgram,
// relaying the simulator's pose and sensor readings to the subprocess and
// applying the motion commands it sends back.
//
// The protocol mirrors the polled shape of the VEX API rather than the
// simulator's promise-based one: motion is started and then watched, so the
// same loops in program.cpp work against a real robot with nothing but the
// IRobot implementation swapped out.

import type { Robot, RobotProgram } from "../src/robot/types";

async function* readLines(stream: ReadableStream<Uint8Array>): AsyncGenerator<string> {
  const decoder = new TextDecoder();
  let buffered = "";
  for await (const chunk of stream) {
    buffered += decoder.decode(chunk, { stream: true });
    for (let brk = buffered.indexOf("\n"); brk >= 0; brk = buffered.indexOf("\n")) {
      yield buffered.slice(0, brk);
      buffered = buffered.slice(brk + 1);
    }
  }
}

export function bridgeProgram(binary: string): RobotProgram {
  return async (r: Robot): Promise<void> => {
    const child = Bun.spawn([binary], { stdin: "pipe", stdout: "pipe", stderr: "inherit" });

    // A motion promise rejects when the run ends underneath the algorithm, by
    // collision or by the goal being captured mid-drive. The subprocess has no
    // exceptions to catch it with, so it becomes a flag it polls instead.
    let moving = false;
    let ended = false;

    const watch = (motion: Promise<void>): void => {
      moving = true;
      motion.then(
        () => {
          moving = false;
        },
        () => {
          moving = false;
          ended = true;
        },
      );
    };

    const settle = async (motion: Promise<void>): Promise<void> => {
      try {
        await motion;
      } catch {
        ended = true;
      }
    };

    const reply = (line: string): void => {
      child.stdin.write(`${line}\n`);
      child.stdin.flush();
    };

    const state = (): string => {
      const p = r.position();
      const measured = r.distance();
      const range = Number.isFinite(measured) ? measured : "inf";
      return `${p.x} ${p.y} ${r.heading()} ${range} ${moving ? 1 : 0} ${ended ? 1 : 0}`;
    };

    try {
      for await (const line of readLines(child.stdout)) {
        const split = line.indexOf(" ");
        const command = split < 0 ? line : line.slice(0, split);
        const argument = split < 0 ? "" : line.slice(split + 1);

        if (command === "DONE") break;
        if (command === "GOAL") {
          const goal = r.goal();
          reply(`${goal.x} ${goal.y} ${goal.radius}`);
          continue;
        }

        switch (command) {
          case "SENSE":
            break;
          case "STEP":
            await settle(r.step());
            break;
          case "TURN":
            await settle(r.turn(Number(argument)));
            break;
          case "STARTDRIVE":
            watch(r.driveFor(Number(argument)));
            break;
          case "STARTTURNTO":
            watch(r.turnTo(Number(argument)));
            break;
          case "LOG":
            r.log(argument);
            break;
          default:
            throw new Error(`unknown bridge command: ${line}`);
        }
        reply(state());
      }
    } finally {
      child.kill();
    }
  };
}
