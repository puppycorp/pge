#!/usr/bin/env bun

import OpenAI from "openai"
import { spawn } from "bun";
import type { ChatCompletionMessageParam, ChatCompletionTool } from "openai/resources/index.mjs";

let apiKey = await Bun.file("./workdir/openai_apikey.txt").text()

const client = new OpenAI({
	apiKey
})

const path = process.argv[2]

const runTests = async (path: string) => {
	// Ensure stdout and stderr are piped
	const proc = Bun.spawn({
		cmd: ["zig", "test", path],
		stdout: "pipe",
		stderr: "pipe",
	});

	// Wait for the process to exit so all data is flushed.
	await proc.exited;

	// Read the complete output from both streams.
	const stdout = await new Response(proc.stdout).text();
	const stderr = await new Response(proc.stderr).text();

	return stdout + "\n" + stderr;
}

// const out = await runTests(path)
// console.log(out)

const systemMsg = `
You are bot which fixes code by running tests and then fixing the code
untill the code is fixed. You can use runTests tool to run the tests
and get the output. But if tests pass dont use runTests tool.
Keep running the tests untill you see no errors anymore!
Dont run tests if all tests success already !
`

const tools: ChatCompletionTool[] = [
	{
		type: "function",
		function: {
			name: "runTests",
			description: "Runs the tests and returns the output of test run"
		}
	},
	{
		type: "function",
		function: {
			name: "writeFile",
			description: "writes new content to file after fixing the code",
			parameters: {
				type: "object",
				properties: {
					content: {
						"type": "string",
						"description": "The content to write to the file",
					}
				}
			}
		}
	}
]

const history: ChatCompletionMessageParam[] =  []

for (let i = 0; i < 5; i++) {
	console.log(`[${i}] iteration`)
	const content = await Bun.file(path).text()
	history.push({
		role: "user",
		content
	})
	const messages: any = [{
		role: "system",
		content: systemMsg
	}, ...history]
	// console.log("messages", messages)
	const res = await client.chat.completions.create({
		model: "gpt-4o-mini",
		messages,
		tools
	})
	const choise = res.choices[0]
	const msg = choise.message
	console.log("msg", msg)
	history.push({
		role: "assistant",
		content: msg.content,
		tool_calls: msg.tool_calls	
	})
	let shouldRunTests = false
	if (msg.tool_calls) {
		for (const call of msg.tool_calls) {
			if (call.function) {
				if (call.function.name === "runTests") {
					shouldRunTests = true
					const content = await runTests(path)
					console.log("test run result", content)
					if (content.includes("All") && content.includes("tests") && content.includes("passed")) {
						shouldRunTests = false
						break
					}
					history.push({
						role: "tool",
						tool_call_id: call.id,
						content
					})
				}
				if (call.function.name === "writeFile") {
					const args = JSON.parse(call.function.arguments)
					const content = args.content
					await Bun.file(path).write(content)
					history.push({
						role: "tool",
						tool_call_id: call.id,
						content: "File written"
					})
				}
			}
		}
	}
	console.log("shouldRunTests", shouldRunTests)
	console.log(msg.content)
	if (!shouldRunTests) break
}

console.log("code fixed")