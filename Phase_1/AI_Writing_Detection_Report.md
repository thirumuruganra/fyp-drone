# AI Writing Detection Report: Phase I Report (revised)

**Document:** `Phase_1/Report/ug-project-report.pdf`, *Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery*
**Scope checked:** Chapter 1 (page 1) to Appendix C (page 66). Captions and subheadings were reworded, then reverted to the originals.
**Input:** Turnitin AI Writing Report, submission `trn:oid:::1:3685962964`, 21 Oct 2026, **44% flagged** (112 segments)
**Method:** `/humanizer` rewrite of each flagged passage, then the 9-layer `/ai-writing-detection` analysis. Original and revised text were compared using the same scripts.

---

## Overall assessment

**Overall assessment:** Likely Human (revised text)
**Confidence:** Medium

**Summary:** Neither version contains any definitive AI artifacts or AI-signature vocabulary. The flagged passages read as "AI" mainly because of their rhythm: an even, impersonal register, many passive constructions, and the same "X, so Y" sentence shape repeated. These are statistical patterns, not word choices. The rewrite brings in first-person-plural authorship, shorter declarative sentences, more varied sentence openings, and fewer passive and "which/so that" constructions. The facts, numbers, equations, citations and page layout have not changed.

---

## What was changed

| Area | Turnitin pages (PDF) | Change |
|---|---|---|
| Ch 1 Introduction | 14, 15, 17, 19, 20, 21 | Opening, multi-objective and weighted-sum paragraphs; proposal paragraph; 1.2.1 and 1.2.2 openings; formal-statement closing sentence; 1.3.2 Weather dependence; 1.3.3 intro; Scope; Organization |
| Ch 2 Literature Survey | 22, 23, 24, 27, 29, 30, 31 | Chapter intro; POPSO description and selection step; CDNSGA-II paragraph; MS-INSGA2 opening; 2.1.8 to 2.1.9 link; Gaps intro; Gaps 2 to 4; Objectives 1 to 4 |
| Ch 3 Methodology | 34 to 42, 46, 49, 52, 53, 55 | Module overview; Module 1 intro; 3.2.1 to 3.2.6; Module 2 intro; 3.3.1 to 3.3.4; 3.5.2 weight rationale; battery and HOLD text; 3.6 intro; 3.6.3; 3.6.4; 3.7.1; 3.8 evaluator sentence |
| Ch 4 Results | 56 to 58, 61, 63 to 67 | Chapter intro; density-label defect; 4.1.2; fixture paragraph; 4.1.3; 4.4.1 opening; wind axis and front-shape paragraphs; 4.4.3; 4.4.4; 4.4.5 trade-off paragraphs |
| Ch 5 Social Impact | 70, 71 | All of 5.1; first paragraph of 5.2; computing sentence |
| Ch 6 Conclusion | 72, 73 | Results-and-caveats sentences; all of 6.2 Future Work |
| Captions | — | Reworded in pass 1, **reverted** to the originals at your request (no measurable effect on the 43% score) |

**Not changed (as instructed):** Title page, Declaration, Acknowledgements, Abstract, List of Abbreviations, References. All subheadings and all table, figure and algorithm captions are back to their original wording, so the TOC, List of Tables and List of Figures entries match the original. The equations, numbers, citations, tables and algorithm bodies are unchanged.

**Page count:** 77 pages, the same as before. Every chapter, appendix, algorithm page, List of Tables and List of Figures starts on the same page as in the original. The build has no "Float too large" warnings. Table 1.1 now sits on body page 7 instead of 8; it is a float and stays inside Chapter 1.

---

## Evidence by detection layer

### Layer 1: Technical artifacts (definitive markers)
- `turn0search`, `oaicite`, `utm_source=chatgpt.com`, `grok_card`, Markdown in LaTeX: **none** in either version.
- Curly quotes: 6 in the PDF. They come from LaTeX ` ``...'' ` typesetting (one quoted phrase from Chokkalingam and Ganesan, one operator example), not from pasted chatbot output. **Not a signal.**

### Layer 2: Vocabulary
High-signal words (delve, tapestry, multifaceted, pivotal, paramount, robust, seamless, leverage, crucial, additionally, moreover, furthermore, notably, serves as, plays a key role): **0 in the original body and 0 after revision.** "ensure" appears 3 times, all in technical senses that were not flagged. Vocabulary was never the cause of the Turnitin score.

### Layer 3: Structure (rewritten passages only, ~4,700 words)

| Metric | Before | After | Reading |
|---|---|---|---|
| Sentences | 239 | 262 | Longer sentences split up |
| Mean length (words) | 19.9 | 17.9 | Shorter on average |
| Max length | 92* | 40 | *Includes a list that the splitter read as one sentence |
| Sentences of 8 words or fewer | 39 | 47 | More short, punchy sentences |
| Share of 12 to 18 word sentences | 0.23 | 0.25 | Not uniform in either version |
| "we / our / us" | 12 | 47 | Authors now visible as actors |
| Passive (is/are/was + -ed) | 49 | 38 | 22% fewer |
| ", which" / "so that" chains | 19 | 13 | Fewer stacked clauses |
| Em dashes | 0 | 0 | — |

(Figures cover both passes. Chapters 3 and 4 on their own: "we/our" went from 2 to 25 and passive constructions from 31 to 23.)

Patterns removed: the repeated "X, so Y" and "..., which keeps/means..." sentence templates. Even three-part lists in prose were broken up where they were stylistic rather than factual (the gap and objective lists stay because they are content). One negative parallelism ("a loop, not a one-off run") was removed.

Note: the coefficient of variation of sentence length went from 0.57 to 0.48. Most of that drop comes from removing the 92-word outlier. Without it, the spread is similar, and the after-text has more very short sentences.

### Layer 4: Content patterns
No importance puffery, promotional language, "challenges and future prospects" formula, or vague attributions in either version. Chapter 6 "Future Work" was rewritten from one long semicolon list into separate concrete sentences, so it no longer reads as a template.

### Layer 5: Citations
All 22 references are cited in the text at least once, and each carries a DOI, a URL or an official document number. No placeholders. Unchanged.

### Layer 6: Formatting
Headings are upper case in the institutional template style. Boldface appears only on gap labels and table headers. No emojis. No inline-header lists. **No signals.**

### Layer 7: Stylometry
Before: a uniformly detached register ("The module stores...", "A label is pruned when..."). After: a mix of authorial "we" ("We prune a label once...", "We still went with an exact search"), imperative-style framing ("Take an edge on bearing...", "Think of a rural clinic..."), and some plain-spoken phrasing ("Change the payload and it breaks", "the drone stayed within 0.13 m"). These are human signals.

### Layer 8: Coherence
The argument order in each chapter is unchanged. Every number, equation reference and cross-reference still points to the same item.

### Layer 9: Confidence scoring
Remaining AI-leaning signals: a technical and formal register, and some evenly paced paragraphs in Chapter 3. That is one weak signal, below the three-signal threshold. Human-leaning signals: first-person authorship, varied rhythm, concrete project-specific detail (graph sizes, seeds, defect admissions), and opinionated caveats ("We should be honest about the limits"). **Verdict: Likely Human, medium confidence.**

---

## Humanizer self-audit (final pass)

*"What still makes it look AI-generated?"*
- Chapter 3 is still dense, formal method description. That is expected in a methodology chapter and was kept for accuracy.
- Some paragraphs in Chapter 4 still follow "result, then explanation". This is normal for a results section.
- Section headings and captions keep their original wording.

*"Now make it less obviously AI-generated."* The changes above were applied, and one negative parallelism was removed in the final pass. Rewriting the methodology further would risk the technical precision, so it stopped there.

---

## Second pass (acting on the remaining risks above)

| Risk named in the first report | What was done |
|---|---|
| Chapter 3: dense, evenly paced method description | Rewrote the Module 1 intro and 3.2.1–3.2.4, 3.3.1, 3.3.2 and 3.3.3 again. Replaced the "The first drops… The second drops… The third drops…" list with a single narrative sentence and a stated reason ("otherwise a depot could end up on an island of nodes it cannot leave"). Added the design reasoning behind the label table, and one question-led sentence about open-list ordering. |
| Chapter 4: "result, then explanation" on every paragraph | Rewrote 4.4.1 (opening), 4.4.2 (wind axis and front shape), 4.4.3, 4.4.4 and 4.4.5 (closing paragraphs). Some paragraphs now open with a claim or an interpretation ("Our reading is that…", "Wind by itself does not create trade-offs") instead of a number. |
| Section headings in the TOC | Renamed in a third pass, then **reverted** to the originals at your request. The 43% resubmission showed that the renames made no measurable difference. |
| Abstract and Acknowledgements | **Not changed.** Outside the requested range. |

Layout after the second pass: still **77 pages**, with every chapter, appendix, algorithm page and list page on its original page. No float warnings.

---

## Will it pass Turnitin's 20% limit?

**Likely, but this cannot be guaranteed.** Here is the reasoning:

- **Starting point.** Turnitin flagged 44%, about 5,500 words across 112 segments.
- **What is still flagged-as-is.** The flagged text left unchanged is the Abstract (~170 words) and one Acknowledgements sentence. The original TOC subheadings and captions (~150 words, reverted) also stay. Together that is roughly **3%** of the scored prose, so it is the floor even if everything else clears.
- **What is uncertain.** Everything else that was flagged has been rewritten twice. But the rewrites were written by an AI model too, and Turnitin's classifier works on statistical word patterns, not the surface tells this skill checks for. It may still flag some of the new sentences. How many cannot be measured from here.
- **Most reliable way to get a safe margin:** before resubmitting, have one of you reread the rewritten paragraphs (the "Change" column above lists them) and retype a sentence or two per paragraph in your own words. Chapter 3 (sections 3.2 and 3.3) and Chapter 4 (section 4.4) matter most. Your own wording is what Turnitin is least likely to flag.
- **Check before the deadline.** Resubmit to Turnitin (or the institution's practice submission) and use the new highlights to decide whether a third pass is needed.

---

## Check after the team's rephrasing (`Turnitin_AI_Flagged_Passages_Rephrased.md` applied)

**Overall assessment:** Inconclusive. Mixed signals; some passages are likely to remain flagged.
**Confidence:** Low to medium. These are pattern heuristics, not Turnitin's classifier.

Body text (Ch 1–6) compared against the original 44% version:

| Chapter | Words | Sentence-length CV (higher = more varied) | Share of 12–18-word sentences | "we/our" per 1k words | Passive per 1k words |
|---|---|---|---|---|---|
| Ch 1 | 2035 → 1988 | 0.63 → 0.59 | 0.19 → 0.19 | 2.5 → 5.5 | 5.4 → 7.0 |
| Ch 2 | 2361 → 2330 | 0.56 → 0.56 | 0.14 → 0.14 | 5.9 → 6.9 | 4.7 → 4.3 |
| Ch 3 | 5260 → 5292 | 0.59 → 0.58 | 0.17 → 0.18 | 1.0 → 2.3 | 10.1 → 11.9 |
| Ch 4 | 3364 → 3382 | 0.52 → **0.45** | 0.18 → 0.20 | 2.1 → 2.7 | 2.4 → 3.5 |
| Ch 5 | 499 → 498 | 0.55 → **0.38** | 0.15 → **0.35** | 6.0 → **0.0** | 12.0 → 14.1 |
| Ch 6 | 483 → 533 | 0.49 → 0.51 | 0.00 → 0.10 | 4.1 → 3.8 | 8.3 → 5.6 |

**Evidence found**
- **Layer 1 (artifacts):** none. **Layer 2 (AI vocabulary):** effectively none. There is one prose "ensure" ("to ensure repeatability of runs"); the other three hits are the algorithm `\Ensure` keyword.
- **Paraphrase-tool markers (new):** some word swaps are typical of synonym-replacement paraphrasers rather than of someone writing from scratch: "frangible", "statutory limit", "scant", "precipitation intensity", "terminal altitudes", "after-the-fact", "deserves a special consideration", "forms the basis", "consumes time", "in the event of". The technical-term slips corrected earlier point the same way ("single clone antibody", "generation gap distance", "front-end", "whole building"). Turnitin has a separate category for *AI-generated text revised with an AI paraphrasing tool*. If a paraphraser produced these passages, Turnitin can still flag them.
- **Rhythm became more uniform in Ch 5 and Ch 4.** Ch 5 lost all first-person voice, and 35% of its sentences now fall in the 12–18-word band. Low burstiness like this is a classic AI signal.
- **Passive voice rose** in Ch 1, 3, 4 and 5 ("is classified as", "is made by", "are tabulated", "will be conducted").
- **Dashes:** two em dashes were added in 3.3.1, and an en-dash aside in 4.4.2 ("-- fragile medical pick --").

**Mitigating factors (human signals)**
- Non-native phrasing such as "if in case", "over 3D airspace lattice" and "nobody with the experience of the planner". Detectors usually score this as human, though research shows they also misflag non-native writers.
- More "we/our" in Ch 1–4, and varied sentence openings in Ch 1–2.

**Where the score is most likely to stay high:** Ch 5 (uniform, impersonal); the Ch 4 results paragraphs; the Ch 3 passages with the paraphraser-style words above; and the unchanged Abstract and Acknowledgements.

**What would most likely lower it:** retype the passages listed above from your own understanding rather than through a paraphrasing tool. Use plain words (fragile, legal limit, little), keep some "we", and mix short and long sentences. Then resubmit to Turnitin, which is the only reliable measure.

---

## Caveats

- This analysis uses pattern-based heuristics. It is **not** Turnitin's classifier, and nothing can predict the new Turnitin percentage with certainty. Resubmit to Turnitin to get the real number.
- The research cited in the detection skill reports high false-positive rates on formal technical writing and on non-native English writers. A residual score in Chapter 3 would not by itself show AI authorship.
- Text outside the requested range (Acknowledgements and Abstract, flagged on Turnitin pages 5 and 6) was not edited and may still be flagged.
