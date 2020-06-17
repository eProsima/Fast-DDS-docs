# Contribution Guidelines

*Fast DDS-Docs* is an open source project for documenting the *Fast DDS* library and some related tools such as *Fast DDS-Gen*.
In that sense, contributions to expand or improve the documentation are most welcome and encouraged; we just want the best documentation possible for out community.

There are two ways to contribute to this project:

* [Opening an issue](#opening-an-issue).
* [Submitting a pull request](#submitting-a-pull-request).

Furthermore, this document also provide the contributor with a [Project structure](#project-structure) definition, and with a [Style guide](#style-guide) to ease the writing process.

## Opening an issue

The easiest way for you to point out an incorrect or missing piece of documentation is to open an issue in the GitHub repository.
When doing so, please take the following into account:

1. Try to use positive and welcoming language.
We sincerely appreciate your contribution and want to help each other.
1. Describe your needs or concerns in detail.
The more information provided, the better our understanding there will be, and the more satisfied you will get in the end.
1. Be patient.
Bear in mind that your issue might not be answered or addressed right away.
If a couple of days go by without any feedback, please send a friendly ping by commenting on the issue.
1. Provide feedback.
Once your issue is addressed, please provide feedback so we can improve the process as we go.

## Submitting a pull request

If writing an issue does not do it for you because you just know what has to be done and how, and you have the energy and time to directly contribute to the documentation, submitting a pull request is your way to go.
Please mind that in doing so, you accept to abide to the pull request process followed on this repository, meaning:

1. Document one thing at a time.
Avoid pull requests where several features or functionalities are documented at the same time.
If you have many contributions, please split them in different pull requests.
1. Submit you pull requests in separate branches.
As for branch names, we normally use `features/*` for documentation expansions or refactors, `hotfix/*` for small fixes, and `bugfix/*` for larger parts that do not reflect reality and need to be corrected.
1. Stick to the [Project structure](#project-structure) and [Style guide](#style-guide).
1. Test your changes before submission, and only submit "green" pull requests.
Our Continuous Integration will still run the tests, but this way the process becomes smoother and the "Fixing errors" commits and re-runs are avoided.
Follow the setup instructions outlined in [README.md](README.md) to prepare your development environment.
If you need any help setting up, please let us know by opening an issue.
We will be happy to help you and to improve this repository's [README.md](README.md) instructions.
1. Be prepared to received criticism, and potentially tons of change requests.
We want our documentation to be as complete, concise, precise, and helpful as possible, so we put a lot of focus on the details.
Please do not take criticism as personal, we really appreciate your contributing effort.
1. Mind that [LICENSE](LICENSE) applies to all contributions made to this repository.

## Project structure

The project follows the following structure:

1. The root directory contains global scope files, such as this one.
1. The [docs directory](#docs-directory) contains all documentation source code.
1. Code snippets and testing code is located in the [code directory](#code-directory).

### docs directory

The [docs](docs) directory contains:

* [_static](docs/_static): For HTML theme related files.
* [01-figures](docs/01-figures): For all the documentation figures. SVG files are the preferred format since the XML can be modified or otherwise checked for differences.
* [02-formalia](docs/02-formalia): Must-have pages such as Introduction.
* [fastdds](docs/fastdds): Fast DDS documentation.
* [fastddsgen](docs/fastddsgen): Fast DDS-Gen documentation.
* [installation](docs/installation): Installation manual.
* [notes](docs/notes): Release notes.

All new documentation must fall into one of these directories, with the exception of those contributions which are not related to any of the given descriptions.
Keep in mind that this is an Sphinx based project, and as such, the all the documentation is written in `reStructuredText`.

All unrecognized words must be added to the [spelling_wordlist.txt](docs/spelling_wordlist.txt) dictionary in alphabetical order, with exception of the ones coming from the API reference documentation, which must be added to [docs/fastdds/api_reference/spelling_wordlist.txt](docs/fastdds/api_reference/spelling_wordlist.txt).

### code directory

The [code](code) directory contains all the files related to code snippets and CI testing.
Files of particular importance are:

* [CodeTester.cpp](code/CodeTester.cpp): Contains all Fast DDS pub-sub and RTPS layer snippets.
It is a buildable file, so it can be compiled and linked against the Fast DDS to verify that all samples of code are up to date.
Furthermore, it is used to create an executable that is then used to generate Ctest tests.
* [DDSCodeTester.cpp](code/DDSCodeTester.cpp): Contains Fast DDS DDS layer snippets.
* [XMLTester.xml](code/XMLTester.xml): Contains XML snippets.
* [doxygen-config.in](code/doxygen-config.in): Doxyfile to configure Doxygen for creating Fast DDS API reference.

## Style guide

This style guide outlines writing directives and dos and don'ts.
Contributors and reviewers should always refer to them when introducing new content so that the text keeps a consistent form throughout the entire documentation.
It is the responsibility of the project maintainers to enforce this guide on every contribution, and to keep it up to date.
The guide is divided in 3 sections:

1. [Writing style](#writing-style)
1. [Text structure](#text-structure)
1. [Content](#content)

### Writing style

1. Use direct and concise language.
Keep in mind that nobody particularly enjoys reading technical documentation, so make your points as clear and fast as possible so the reader can quickly get what they need and move on.
1. Use reported speech when writing the documentation, as it is the most suitable mode for formal technical English.
1. In this sense, avoid contractions of any kind.
1. Direct speech may be used on tutorials, but it should be replaced with imperative mode with no subject when possible.
1. Always state what an acronym stands for the first time you use it.
Try to also state it from time to time as you go.

### Text structure

1. Go from the abstract to the concrete.
Give a big picture first and then start digging into the specifics.
1. Always write keeping maintainability in mind.
1. Split you content in different pages as much as readability allows.
This makes it easier for everyone to find just the piece of information they need without scrolling back and forth over endlessly long pages.
1. Tables are a great form of comprising a lot of data in a small, easy to consult manner.
Use them as much as you can, even if it duplicates some information.
1. Use as many `note`, `important`, and `warning` blocks as you need.
They are a great way to convey a really important message the reader should not overlook.
1. Add cross-references to other relevant sections, API reference, and external sites or documents as much as possible.
Make sure the reader has easy access to all relevant information they may need for fully comprehending the documentation.
1. Avoid two titles without text in the middle.
If this happens, it probably means that your are missing some introductory text.
1. Avoid sections with only one subsection as much as possible.
If your section only has one subsection, then the section and subsection are probably one and the same.
1. If your section has multiple subsections, include a list of them at the end of your introduction, specially if not all the subsections fit in your screen at the same time.
This way you provide the user with an overview of what you are documenting, making the navigation easier.
1. Add unique, understandable labels to all your sections, you do not know when might they be referenced.

### Content

1. Always write thinking that the user does not know anything about the topic.
Odds are they do not.
1. Do not write pretending the user has read all your previous sections.
Odds are they have not, and they probably will not.
1. In that sense, cross-reference when some previous content is needed.
1. Provide as many high level figures, class diagram, and any other supporting material as needed.
It is always easier to understand complex topics with a diagram, plus it will help you to structure your section.
1. Do not get to much into implementation details.
After all, this is a user documentation, so it should document API, functionalities, and how to use and configure the middleware in general.
The user does not need to know how things are done under the hood.
1. Provide lots of heavily commented code snippets.
The C++ and XML code samples must be taken from the source files in the [code](code) directory and never be included under a code-block.
This way, the code is tested and we can guarantee it usability.
1. Code snippets must be copy-paste ready, meaning that they should provide all the context necessary for a user to just copy-paste them in their projects.
1. For the case of XML snippets, they must constitute a fully functional XML configuration file.
