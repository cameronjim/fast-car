import { describe, expect, it, vi } from "vitest";
import { Api, ApiError, loginError, type TokenStore } from "./api";
import { externalDriveText, gapSpeed, pnpSelectionText, selectedBundle, selectedParamsFile } from "./drivePick";

describe("bundle selection", () => {
  it("selects the only bundle, so the request names it", () => {
    expect(selectedBundle("", ["dyn-v0"])).toBe("dyn-v0");
  });

  it("selects nothing when there are several and none is picked", () => {
    expect(selectedBundle("", ["a", "b"])).toBe("");
  });

  it("keeps a pick that is still offered and drops one that is not", () => {
    expect(selectedBundle("b", ["a", "b"])).toBe("b");
    expect(selectedBundle("gone", ["a", "b"])).toBe("");
    expect(selectedBundle("gone", ["a"])).toBe("a");
  });

  it("selects nothing when there is no bundle at all", () => {
    expect(selectedBundle("", [])).toBe("");
    expect(selectedBundle("x", undefined)).toBe("");
  });

  it("only passes a planner file that was offered", () => {
    expect(selectedParamsFile("planner-2026-11-12.yaml", ["planner-2026-11-12.yaml"])).toBe("planner-2026-11-12.yaml");
    expect(selectedParamsFile("../etc/passwd", ["planner-2026-11-12.yaml"])).toBe("");
    expect(selectedParamsFile("", undefined)).toBe("");
  });

  it("says what will launch, naming the bundle", () => {
    const text = pnpSelectionText({
      backend: "geometric",
      dynamics: "dyn-v0",
      perception: "",
      paramsFile: "planner-2026-11-12.yaml",
      speed: 0.5,
    });
    expect(text).toContain("dynamics bundle: dyn-v0");
    expect(text).toContain("planner file: planner-2026-11-12.yaml");
    expect(text).not.toContain("perception");
    expect(pnpSelectionText({ backend: "net", dynamics: "", perception: "", paramsFile: "", speed: 0.5 })).toContain(
      "dynamics bundle: NONE selected, perception bundle: NONE selected, planner file: default (none)",
    );
  });
});

describe("gap follow max speed", () => {
  const bounds = { min: 0.36, max: 1.0 };

  it("empty means the profile's own speed", () => {
    expect(gapSpeed("", bounds)).toEqual({ value: undefined, error: "" });
    expect(gapSpeed("  ", bounds)).toEqual({ value: undefined, error: "" });
  });

  it("accepts a number inside the bounds, edges included", () => {
    expect(gapSpeed("0.6", bounds).value).toBe(0.6);
    expect(gapSpeed("0.36", bounds).value).toBe(0.36);
    expect(gapSpeed("1", bounds).value).toBe(1);
  });

  it("refuses a number outside the bounds, or not a number", () => {
    for (const bad of ["0.35", "1.01", "-1", "0"]) expect(gapSpeed(bad, bounds).error).toContain("between 0.36 and 1");
    expect(gapSpeed("fast", bounds).error).toContain("not a number");
    expect(gapSpeed("Infinity", bounds).error).toContain("not a number");
  });

  it("with no bounds known yet, a number passes (the agent still checks)", () => {
    expect(gapSpeed("0.5", undefined).value).toBe(0.5);
  });
});

describe("terminal drives", () => {
  it("says nothing when there is none", () => {
    expect(externalDriveText(undefined)).toBeNull();
    expect(externalDriveText([])).toBeNull();
  });

  it("names each one and says STOP ALL stops it", () => {
    const text = externalDriveText([{ pid: 77, mode: "gap_follow" }, { pid: 90, mode: "pnp" }]);
    expect(text).toContain("gap_follow (pid 77), pnp (pid 90)");
    expect(text).toContain("STOP ALL stops it");
  });
});

function memoryTokens(initial: string | null): TokenStore & { value: string | null } {
  const store = {
    value: initial,
    get: () => store.value,
    set: (t: string) => void (store.value = t),
    clear: () => void (store.value = null),
  };
  return store;
}

describe("verifying a token before it is saved", () => {
  it("sends the candidate token, not the saved one, and stores nothing", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(new Response("{}", { status: 200 }));
    const tokens = memoryTokens("saved");
    await new Api(tokens, fetchImpl).verifyToken("  candidate \n");
    expect(fetchImpl.mock.calls[0][1].headers.Authorization).toBe("Bearer candidate");
    expect(tokens.value).toBe("saved");
  });

  it("a rejected candidate throws 401 without the unauthorized callback or touching the saved token", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(new Response(JSON.stringify({ error: "nope" }), { status: 401 }));
    const tokens = memoryTokens("saved");
    const api = new Api(tokens, fetchImpl);
    const onUnauthorized = vi.fn();
    api.onUnauthorized = onUnauthorized;
    await expect(api.verifyToken("wrong")).rejects.toMatchObject({ status: 401 });
    expect(onUnauthorized).not.toHaveBeenCalled();
    expect(tokens.value).toBe("saved");
  });

  it("an unreachable car is a network error, not a rejected token", async () => {
    const api = new Api(memoryTokens(null), vi.fn().mockRejectedValue(new Error("down")));
    await expect(api.verifyToken("x")).rejects.toBeInstanceOf(ApiError);
    await expect(api.verifyToken("x")).rejects.toMatchObject({ status: 0 });
  });
});

describe("login messages", () => {
  it("tells a rejected token from a lockout from no connection", () => {
    expect(loginError(new ApiError("x", 401, null))).toContain("not accepted");
    expect(loginError(new ApiError("x", 429, null))).toContain("Wait a minute");
    expect(loginError(new ApiError("x", 0, null))).toContain("Cannot reach the car");
    expect(loginError(new Error("boom"))).toBe("Error: boom");
  });
});
