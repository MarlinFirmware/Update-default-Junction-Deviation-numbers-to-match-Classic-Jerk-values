# Contributing to Marlin

Thanks for your interest in contributing to Marlin Firmware!

The following is a set of guidelines for contributing to Marlin, hosted by the [MarlinFirmware Organization](https://github.com/MarlinFirmware) on GitHub. These are mostly guidelines, not rules. Use your best judgment, and feel free to propose changes to this document in a Pull Request.

#### Table Of Contents

[Code of Conduct](#code-of-conduct)

[I don't want to read this whole thing, I just have a question!!!](#i-dont-want-to-read-this-whole-thing-i-just-have-a-question)

[How Can I Contribute?](#how-can-i-contribute)
  * [Reporting Bugs](#reporting-bugs)
  * [Suggesting Features or Changes](#suggesting-features-or-changes)
  * [Your First Code Contribution](#your-first-code-contribution)
  * [Pull Requests](#pull-requests)

[Styleguides](#styleguides)
  * [Git Commit Messages](#git-commit-messages)
  * [C++ Coding Standards](#c++-coding-standards)
  * [Documentation Styleguide](#documentation)

[Additional Notes](#additional-notes)
  * [Issue and Pull Request Labels](#issue-and-pull-request-labels)

## Code of Conduct

This project and everyone participating in it is governed by the [Marlin Code of Conduct](code_of_conduct.md). By participating, you are expected to uphold this code. Please report unacceptable behavior by following GitHub's [reporting abuse or spam article](https://docs.github.com/en/communities/maintaining-your-safety-on-github/reporting-abuse-or-spam).

## I don't want to read this whole thing I just have a question!!!

> [!NOTE]
> Please don't file an issue to ask a question. You'll get faster results by using the resources below.

We have a Message Board and a Facebook group where our knowledgable user community can provide helpful advice if you have questions.

- [Marlin Documentation](https://marlinfw.org) - Official Marlin documentation
- Facebook Group ["Marlin Firmware"](https://www.facebook.com/groups/1049718498464482/)
- RepRap.org [Marlin Forum](https://forums.reprap.org/list.php?415)
- Facebook Group ["Marlin Firmware for 3D Printers"](https://www.facebook.com/groups/3Dtechtalk/)
- [Marlin Configuration](https://www.youtube.com/results?search_query=marlin+configuration) on YouTube

If chat is more your speed, you can join the MarlinFirmware Discord server:

* Use the link https://discord.com/servers/marlin-firmware-461605380783472640 to join up as a General User.
* Even though our Discord is pretty active, it may take a while for community members to respond &mdash; please be patient!
* Use the `#general` channel for general questions or discussion about Marlin.
* Other channels exist for certain topics or are limited to Patrons. Check the channel list.

## How Can I Contribute?

### Reporting Bugs

This section guides you through submitting a Bug Report for Marlin. Following these guidelines helps maintainers and the community understand your report, reproduce the behavior, and find related reports.

Before creating a Bug Report, please test the "nightly" development branch, as you might find out that you don't need to create one. When you are creating a Bug Report, please [include as many details as possible](#how-do-i-submit-a-good-bug-report). Fill out [the required template](ISSUE_TEMPLATE/bug_report.yml), the information it asks for helps us resolve issues faster.

> [!NOTE]
> Regressions can happen. If you find a **Closed** issue that seems like your issue, go ahead and open a new issue and include a link to the original issue in the body of your new one. All you need to create a link is the issue number, preceded by #. For example, #8888.

#### How Do I Submit A (Good) Bug Report?

Bugs are tracked as [GitHub issues](https://guides.github.com/features/issues/). Use the New Issue button to create an issue and provide the following information by filling in [the template](ISSUE_TEMPLATE/bug_report.yml).

Explain the problem and include additional details to help maintainers reproduce the problem:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. For example, start by explaining how you started Marlin, e.g. which command exactly you used in the terminal, or how you started Marlin otherwise. When listing steps, **don't just say what you did, but explain how you did it**. For example, if you moved the cursor to the end of a line, explain if you used the mouse, or a keyboard shortcut or an Marlin command, and if so which one?
* **Provide specific examples to demonstrate the steps**. Include links to files or GitHub projects, or copy/pasteable snippets, which you use in those examples. If you're providing snippets or log output in the issue, use [Markdown code blocks](https://help.github.com/articles/markdown-basics/#multiple-lines).
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why.**
* **Include detailed log output** especially for probing and leveling. See below for usage of `DEBUG_LEVELING_FEATURE`.
* **Include screenshots, links to videos, etc.** which clearly demonstrate the problem.
* **Include G-code** (if relevant) that reliably causes the problem to show itself.
* **If the problem wasn't triggered by a specific action**, describe what you were doing before the problem happened and share more information using the guidelines below.

Provide more context:

* **Can you reproduce the problem with a minimum of options enabled?**
* **Did the problem start happening recently** (e.g. after updating to a new version of Marlin) or was this always a problem?
* If the problem started happening recently, **can you reproduce the problem in an older version of Marlin?** What's the most recent version in which the problem doesn't happen? You can download older versions of Marlin from [the releases page](https://github.com/MarlinFirmware/Marlin/releases).
* **Can you reliably reproduce the issue?** If not, provide details about how often the problem happens and under which conditions it normally happens.

Include details about your configuration and environment:

* **Which version of Marlin are you using?** Marlin's exact version and build date can be seen in the startup message when a host connects to Marlin, or in the LCD Info menu (if enabled).
* **What kind of 3D Printer and electronics are you using**?
* **What kind of add-ons (probe, filament sensor) do you have**?
* **Include your Configuration files.** Make a ZIP file containing `Configuration.h` and `Configuration_adv.h` and drop it on your reply.

### Suggesting Features or Changes

This section guides you through submitting a suggestion for Marlin, including completely new features and minor improvements to existing functionality. Following these guidelines helps maintainers and the community understand your suggestion and find related suggestions.

Before creating a suggestion, please check [this list](https://github.com/MarlinFirmware/Marlin/issues?q=is%3Aopen+is%3Aissue+label%3A%22T%3A+Feature+Request%22) as you might find out that you don't need to create one. When you are creating an enhancement suggestion, please [include as many details as possible](#how-do-i-submit-a-good-feature-request). Fill in [the template](ISSUE_TEMPLATE/feature_request.yml), including the steps that you imagine you would take if the feature you're requesting existed.

#### Before Submitting a Feature Request

* **Check the [Marlin website](https://marlinfw.org/)** for tips — you might discover that the feature is already included. Most importantly, check if you're using [the latest version of Marlin](https://github.com/MarlinFirmware/Marlin/releases) and if you can get the desired behavior by changing [Marlin's config settings](https://marlinfw.org/docs/configuration/configuration.html).
* **Perform a [cursory search](https://github.com/MarlinFirmware/Marlin/issues?q=is%3Aopen+is%3Aissue+label%3A%22T%3A+Feature+Request%22)** to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit A (Good) Feature Request?

Feature Requests are tracked as [GitHub issues](https://guides.github.com/features/issues/). Please follow these guidelines in your request:

* **Use a clear and descriptive title** for the issue to identify the suggestion.
* **Provide a step-by-step description of the requested feature** in as much detail as possible.
* **Provide specific examples to demonstrate the steps**.
* **Describe the current behavior** and **explain which behavior you expected to see instead** and why.
* **Include screenshots and links to videos** which demonstrate the feature or point out the part of Marlin to which the request is related.
* **Explain why this feature would be useful** to most Marlin users.
* **Name other firmwares that have this feature, if any.**

### Your First Code Contribution

Unsure where to begin contributing to Marlin? You can start by looking through these `good-first-issue` and `help-wanted` issues:

* [Beginner issues][good-first-issue] - issues which should only require a few lines of code, and a test or two.
* [Help Wanted issues][help-wanted] - issues which should be a bit more involved than `beginner` issues.

### Pull Requests

Pull Requests should always be targeted to working branches (e.g., `bugfix-2.1.x` and/or `bugfix-1.1.x`) and never to release branches (e.g., `2.0.x` and/or `1.1.x`). If this is your first Pull Request, please read our [Guide to Pull Requests](https://marlinfw.org/docs/development/getting_started_pull_requests.html) and Github's [Pull Request](https://help.github.com/articles/creating-a-pull-request/) documentation.

* Fill in [the required template](pull_request_template.md).
* Don't include issue numbers in the PR title.
* Include pictures, diagrams, and links to videos in your Pull Request to demonstrate your changes, if needed.
* Follow the [Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) posted on our website.
* Document new code with clear and concise comments.
* End all files with a newline.

## Styleguides

### Git Commit Messages

* Use the present tense ("Add feature" not "Added feature").
* Use the imperative mood ("Move cursor to..." not "Moves cursor to...").
* Limit the first line to 72 characters or fewer.
* Reference issues and Pull Requests liberally after the first line.

### C++ Coding Standards

* Please read and follow the [Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) posted on our website. Failure to follow these guidelines will delay evaluation and acceptance of Pull Requests.

### Documentation

* Guidelines for documentation are still under development. In-general, be clear, concise, and to-the-point.
