#!/usr/bin/env bun

import readline from "readline";
import { spawnSync } from "child_process";
import OpenAI from "openai"
import { spawn } from "bun";

let apiKey = await Bun.file("./workdir/openai_apikey.txt").text()

const client = new OpenAI({
	apiKey
})

const path = process.argv[2]

const runTests = async (path: string) => {
	const proc = spawn(["zig", "test", path])
	const output = await new Response(proc.stdout).text();
	console.log("Tests ran with result", output)
}

for (let i = 0; i < 5; i++) {
	console.log(`[${i}] iteration`)
	await runTests(path)
}


// // Create a readline interface for interactive input
// const rl = readline.createInterface({
//   input: process.stdin,
//   output: process.stdout,
// });

// // Prompt for the original file path
// rl.question("Enter the path for the original file: ", (originalFile) => {
//   // Prompt for the modified file path
//   rl.question("Enter the path for the modified file: ", (modifiedFile) => {
//     // Launch VS Code diff using the provided file paths
//     const result = spawnSync("code", ["--diff", originalFile, modifiedFile], {
//       stdio: "inherit",
//     });

//     if (result.error) {
//       console.error("Error launching VS Code diff:", result.error);
//     } else {
//       console.log("VS Code diff view launched successfully.");
//     }

//     // Close the readline interface
//     rl.close();
//   });
// });