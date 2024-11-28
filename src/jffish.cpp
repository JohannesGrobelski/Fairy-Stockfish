/* This Wrapper exposes fairy stockfish api for java.
 * It is heavily influenced by pyffish.cpp, and implements most of its logic for a java binding.
 */
#include <jni.h>
#include <string>
#include <sstream>
#include <vector>

#include "misc.h"
#include "types.h"
#include "bitboard.h"
#include "evaluate.h"
#include "position.h"
#include "search.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"
#include "piece.h"
#include "variant.h"
#include "apiutil.h"

using namespace Stockfish;
using namespace std;

/**
* Builds a chess position from given parameters
* @param pos Position object to initialize
* @param states StateInfo list for move history
* @param variant Chess variant name
* @param fen Starting FEN string
* @param moveList List of moves to apply
* @param chess960 Chess960 mode flag
* @details
* - Initializes position with variant and FEN
* - Applies each move in moveList sequentially
* - Maintains move history in states
*/
void buildPosition(Position& pos, StateListPtr& states, const char* variant, const char* fen,
                   const vector<string>& moveList, bool chess960) {

    states = StateListPtr(new deque<StateInfo>(1)); // Drop old and create a new one

    const Variant* v = variants.find(string(variant))->second;
    UCI::init_variant(v);
    if (strcmp(fen, "startpos") == 0)
        fen = v->startFen.c_str();
    pos.set(v, string(fen), chess960, &states->back(), Threads.main());

    //parse move list
    for (auto moveStr : moveList) {
        Move m = UCI::to_move(pos, moveStr);
        if (m != MOVE_NONE) {
            states->emplace_back();
            pos.do_move(m, states->back());
        }
    }
}

//API
//Note: registering methods isn't necessary in JNI as long as you follow the standard JNI naming convention: Java_package_class_method. The Java runtime automatically links these methods by name.
extern "C" {
    /**
    * Returns the version of the Fairy-Stockfish library
    * @return int[3] array containing major, minor, patch version
    */
    JNIEXPORT jintArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_version(JNIEnv* env, jobject) {
        jintArray result = env->NewIntArray(3);
        jint version[3] = {0, 0, 84};
        env->SetIntArrayRegion(result, 0, 3, version);
        return result;
    }

    /**
     * Returns information about the Fairy-Stockfish engine
     * @return String containing engine info
     */
    JNIEXPORT jstring JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_info(JNIEnv* env, jobject) {
        return env->NewStringUTF(engine_info().c_str());
    }

    /**
     * Gets list of supported chess variants
     * @return String array of variant names
     */
    JNIEXPORT jobjectArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_variants(JNIEnv* env, jobject) {
        vector<string> vars = variants.get_keys();
        jobjectArray result = env->NewObjectArray(vars.size(),
                                                  env->FindClass("java/lang/String"), nullptr);
        for(size_t i = 0; i < vars.size(); i++) {
            env->SetObjectArrayElement(result, i,
                                       env->NewStringUTF(vars[i].c_str()));
        }
        return result;
    }

    /**
     * Sets a UCI option
     * @param name Option name
     * @param value Option value
     * @throws IllegalArgumentException if option doesn't exist
     */
    JNIEXPORT void JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_setOption(JNIEnv* env, jobject,
                                                  jstring name, jstring value) {
        const char* nameStr = env->GetStringUTFChars(name, nullptr);
        const char* valueStr = env->GetStringUTFChars(value, nullptr);

        if (Options.count(nameStr)) {
            Options[nameStr] = string(valueStr);
        } else {
            env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"),
                          (string("No such option ") + nameStr).c_str());
        }

        env->ReleaseStringUTFChars(name, nameStr);
        env->ReleaseStringUTFChars(value, valueStr);
    }

    /**
     * Loads variant configuration
     * @param config Variant configuration string
     */
    JNIEXPORT void JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_loadVariantConfig(JNIEnv* env, jobject, jstring config) {
        const char* configStr = env->GetStringUTFChars(config, nullptr);
        stringstream ss(configStr);
        variants.parse_istream<false>(ss);
        Options["UCI_Variant"].set_combo(variants.get_keys());
        env->ReleaseStringUTFChars(config, configStr);
    }

    /**
     * Gets starting FEN position for a variant
     * @param variant Variant name
     * @return Starting FEN string
     */
    JNIEXPORT jstring JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_startFen(JNIEnv* env, jobject, jstring variant, jboolean chess960) {
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        jstring result = env->NewStringUTF(variants.find(string(variantStr))->second->startFen.c_str());
        env->ReleaseStringUTFChars(variant, variantStr);
        return result;
    }

    /**
     * Checks if variant uses two boards
     * @param variant Variant name
     * @return true if variant uses two boards
     */
    JNIEXPORT jboolean JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_twoBoards(JNIEnv* env, jobject, jstring variant) {
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        jboolean result = variants.find(string(variantStr))->second->twoBoards;
        env->ReleaseStringUTFChars(variant, variantStr);
        return result;
    }

    /**
     * Checks if captures go to hand in variant
     * @param variant Variant name
     * @return true if captures go to hand
     */
    JNIEXPORT jboolean JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_capturesToHand(JNIEnv* env, jobject, jstring variant) {
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        jboolean result = variants.find(string(variantStr))->second->capturesToHand;
        env->ReleaseStringUTFChars(variant, variantStr);
        return result;
    }

    /**
     * Gets list of legal moves from position
     * @param variant Variant name
     * @param fen Position FEN
     * @param moves Previous moves list
     * @param chess960 Chess960 mode flag
     * @return Array of legal moves in UCI notation
     */
    JNIEXPORT jobjectArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_legalMoves(JNIEnv* env, jobject,
                                                   jstring variant, jstring fen, jboolean chess960) {

        Position pos;
        StateListPtr states(new deque<StateInfo>(1));
        vector<string> moveList;

        // Convert Java moves array to vector
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);

        vector<string> legalMoves;
        for(const auto& m : MoveList<LEGAL>(pos)) {
            legalMoves.push_back(UCI::move(pos, m));
        }

        jobjectArray result = env->NewObjectArray(legalMoves.size(),
                                                  env->FindClass("java/lang/String"), nullptr);

        for(size_t i = 0; i < legalMoves.size(); i++) {
            env->SetObjectArrayElement(result, i,
                                       env->NewStringUTF(legalMoves[i].c_str()));
        }

        env->ReleaseStringUTFChars(variant, variantStr);
        env->ReleaseStringUTFChars(fen, fenStr);

        return result;
    }

    /**
    * Checks if position is check
    * @param variant Variant name
    * @param fen Position FEN
    * @param moves Previous moves
    * @param chess960 Chess960 mode flag
    * @return true if position is check
    */
    JNIEXPORT jboolean JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_givesCheck(JNIEnv* env, jobject,
                                                   jstring variant, jstring fen, jobjectArray moves, jboolean chess960) {
        Position pos;
        StateListPtr states(new deque<StateInfo>(1));
        vector<string> moveList;

        jsize len = env->GetArrayLength(moves);
        for(jsize i = 0; i < len; i++) {
            auto move = (jstring)env->GetObjectArrayElement(moves, i);
            moveList.push_back(env->GetStringUTFChars(move, nullptr));
        }

        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);
        jboolean result = Stockfish::checked(pos);

        env->ReleaseStringUTFChars(variant, variantStr);
        env->ReleaseStringUTFChars(fen, fenStr);

        return result;
    }

    /**
    * Gets game result
    * @return 1 for white win, -1 for black win, 0 for draw
    */
    JNIEXPORT jint JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_getGameResult(JNIEnv* env, jobject) {
        Position pos;
        Value result;
        if (pos.is_immediate_game_end(result)) {
            return result > VALUE_DRAW ? 1 : (result < VALUE_DRAW ? -1 : 0);
        }
        if (pos.checkers()) {
            return pos.checkmate_value() > VALUE_DRAW ? 1 : -1;
        }
        return 0; // Draw
    }

    /**
    * Checks if move is a capture
    * @param variant Variant name
    * @param fen Position FEN
    * @param moves Previous moves
    * @param move Move to check
    * @param chess960 Chess960 mode flag
    * @return true if move is capture
    */
    JNIEXPORT jboolean JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_isCapture(JNIEnv* env, jobject,
                                                  jstring variant, jstring fen, jstring move, jboolean chess960) {
        Position pos;
        StateListPtr states(new deque<StateInfo>(1));
        vector<string> moveList;

        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);
        const char* moveStr = env->GetStringUTFChars(move, nullptr);
        auto move_str = std::string(env->GetStringUTFChars(move, nullptr));

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);
        jboolean result = pos.capture(UCI::to_move(pos,move_str));

        env->ReleaseStringUTFChars(variant, variantStr);
        env->ReleaseStringUTFChars(fen, fenStr);
        env->ReleaseStringUTFChars(move, moveStr);

        return result;
    }

    /**
    * Checks if game is immediately ended
    * @param variant Variant name
    * @param fen Position FEN
    * @param moves Move list
    * @param chess960 Chess960 mode flag
    * @return Array [isGameEnd, result]
    */
    JNIEXPORT jintArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_isImmediateGameEnd(JNIEnv* env, jobject,
                                                           jstring variant, jstring fen, jboolean chess960) {
        Position pos;
        StateListPtr states(new deque<StateInfo>(1));
        vector<string> moveList;
        bool gameEnd;
        Value result;

        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);
        gameEnd = pos.is_immediate_game_end(result);

        jintArray jresult = env->NewIntArray(2);
        jint values[2] = {gameEnd, result};
        env->SetIntArrayRegion(jresult, 0, 2, values);

        return jresult;
    }

    /**
    * Checks if material is insufficient
    * @param variant Variant name
    * @param fen Position FEN
    * @param moves Move list
    * @param chess960 Chess960 mode flag
    * @return Array [whiteInsufficient, blackInsufficient]
    */
    JNIEXPORT jbooleanArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_hasInsufficientMaterial(JNIEnv* env, jobject,
                                                                jstring variant, jstring fen, jobjectArray moves, jboolean chess960) {
        Position pos;
        StateListPtr states(new deque<StateInfo>(1));
        vector<string> moveList;

        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);

        bool wInsufficient = has_insufficient_material(WHITE, pos);
        bool bInsufficient = has_insufficient_material(BLACK, pos);

        jbooleanArray result = env->NewBooleanArray(2);
        jboolean values[2] = {wInsufficient, bInsufficient};
        env->SetBooleanArrayRegion(result, 0, 2, values);

        return result;
    }

    /**
    * Validates FEN string for variant
    * @param fen FEN to validate
    * @param variant Variant name
    * @param chess960 Chess960 mode flag
    * @return Validation result code
    */
    JNIEXPORT jint JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_validateFen(JNIEnv* env, jobject,
                                                    jstring fen, jstring variant, jboolean chess960) {
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);

        jint result = FEN::validate_fen(string(fenStr),
                                        variants.find(string(variantStr))->second, chess960);

        env->ReleaseStringUTFChars(fen, fenStr);
        env->ReleaseStringUTFChars(variant, variantStr);

        return result;
    }

    JNIEXPORT jstring JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_getFEN(JNIEnv *env, jobject obj,
                                  jstring variant,
                                  jstring fen,
                                  jobjectArray moveList,
                                  jboolean chess960,
                                  jboolean sfen,
                                  jboolean showPromoted,
                                  jint countStarted) {
        // Convert Java strings to C++ strings
        const char *variantCStr = env->GetStringUTFChars(variant, nullptr);
        const char *fenCStr = env->GetStringUTFChars(fen, nullptr);

        // Extract move list from Java array
        std::vector<std::string> moves;
        jsize moveListLength = env->GetArrayLength(moveList);
        for (jsize i = 0; i < moveListLength; i++) {
            jstring move = (jstring) env->GetObjectArrayElement(moveList, i);
            const char *moveCStr = env->GetStringUTFChars(move, nullptr);
            moves.emplace_back(moveCStr);
            env->ReleaseStringUTFChars(move, moveCStr);
            env->DeleteLocalRef(move);
        }

        // Initialize position and states
        Position pos;
        StateListPtr states(new std::deque<StateInfo>(1));

        // Build position
        buildPosition(pos, states, variantCStr, fenCStr, moves, chess960);

        // Get the resulting FEN string
        std::string resultFEN = pos.fen(sfen, showPromoted, countStarted);

        // Release memory for input strings
        env->ReleaseStringUTFChars(variant, variantCStr);
        env->ReleaseStringUTFChars(fen, fenCStr);

        // Return the FEN as a Java string
        return env->NewStringUTF(resultFEN.c_str());
    }

    /**
    * Calculates best move for given position
    * @param variant Variant name
    * @param fen Position FEN
    * @param moves Previous moves
    * @param moveTime Time to think in milliseconds
    * @return Best move in UCI notation
    */
    JNIEXPORT jstring JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_calcBestMove(JNIEnv* env, jobject,
                                                     jstring variant, jstring fen, int depth, int movetime, jboolean chess960) {
        Position pos;
        StateListPtr states(new std::deque<StateInfo>(1));
        std::vector<std::string> moveList;

        const char* variantStr = env->GetStringUTFChars(variant, nullptr);
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);

        buildPosition(pos, states, variantStr, fenStr, moveList, chess960);

        // Set search parameters
        Search::LimitsType limits;
        limits.movetime = movetime;
        limits.depth = depth;

        Threads.start_thinking(pos, states, limits);
        Threads.main()->wait_for_search_finished();

        Move bestMove = Threads.main()->rootMoves[0].pv[0];
        return env->NewStringUTF(UCI::move(pos, bestMove).c_str());
    }

    /**
    * Sets position from FEN string
    */
    JNIEXPORT void JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_setPosition(JNIEnv* env, jobject, jstring fen) {
        Position pos;
        StateListPtr states(new std::deque<StateInfo>(1));
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);
        buildPosition(pos, states, "chess", fenStr, std::vector<std::string>(), false);
        env->ReleaseStringUTFChars(fen, fenStr);
    }

    /**
     * Gets piece and color at square
     * @return String array [piece, color]
     */
    JNIEXPORT jobjectArray JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_getPiece(JNIEnv* env, jobject, jint file, jint rank) {
        Position pos;
        Piece piece = pos.piece_on(make_square(File(file), Rank(rank)));

        jobjectArray result = env->NewObjectArray(2, env->FindClass("java/lang/String"), nullptr);
        env->SetObjectArrayElement(result, 0, env->NewStringUTF(piece_name(type_of(piece)).c_str()));
        env->SetObjectArrayElement(result, 1, env->NewStringUTF(color_of(piece) == WHITE ? "white" : "black"));
        return result;
    }

    /**
     * Checks if move is legal
     */
    JNIEXPORT jboolean JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_isLegalMove(JNIEnv* env, jobject,
                                                                    jstring variant, jstring fen, jstring move, jboolean chess960) {
        Position pos;
        StateListPtr states(new std::deque<StateInfo>(1));
        const char* fenStr = env->GetStringUTFChars(fen, nullptr);
        const char* moveStr = env->GetStringUTFChars(move, nullptr);
        auto move_str = std::string(env->GetStringUTFChars(move, nullptr));
        const char* variantStr = env->GetStringUTFChars(variant, nullptr);

        buildPosition(pos, states, variantStr, fenStr, {}, chess960);
        Move m = UCI::to_move(pos, move_str);
        jboolean result = MoveList<LEGAL>(pos).contains(m);

        env->ReleaseStringUTFChars(fen, fenStr);
        env->ReleaseStringUTFChars(move, moveStr);
        return result;
    }

    /**
    * Gets current FEN string of position
    */
    JNIEXPORT jstring JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_getCurrentFEN(JNIEnv* env, jobject) {
        Position pos;
        jstring result = env->NewStringUTF(pos.fen().c_str());
        return result;
    }

    /**
    * Initializes Stockfish engine
    */
    JNIEXPORT jint JNICALL
    Java_emerald_apps_fairychess_model_board_Chessboard_initEngine(JNIEnv* env, jobject obj) {
        pieceMap.init();
        variants.init();
        UCI::init(Options);
        PSQT::init(variants.find(Options["UCI_Variant"])->second);
        Bitboards::init();
        Position::init();
        Bitbases::init();
        Search::init();
        Threads.set(Options["Threads"]);
        Search::clear();
        return 0;
    }
}
// end of extern "C"

//initialization
//env and obj are JNIEnv pointers and provide interface to JNI functions
#define JNIEXPORT_INIT(name) JNIEXPORT void JNICALL Java_emerald_apps_fairychess_model_board_Chessboard_##name(JNIEnv* env, jobject obj)

JNIEXPORT_INIT(init) {
    // Initialize constants as static fields, these will later available after load jffish library (e.g. VALUE_MATE available as public static int VALUE_MATE;)
    // Gets jclass reference (Java class) to set static fields (env and obj)
    jclass cls = env->GetObjectClass(obj);

    //values
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "VALUE_MATE", "I"), VALUE_MATE);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "VALUE_DRAW", "I"), VALUE_DRAW);

    //notations
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_DEFAULT", "I"), NOTATION_DEFAULT);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_SAN", "I"), NOTATION_SAN);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_LAN", "I"), NOTATION_LAN);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_SHOGI_HOSKING", "I"), NOTATION_SHOGI_HOSKING);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_SHOGI_HODGES", "I"), NOTATION_SHOGI_HODGES);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_SHOGI_HODGES_NUMBER", "I"), NOTATION_SHOGI_HODGES_NUMBER);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_JANGGI", "I"), NOTATION_JANGGI);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_XIANGQI_WXF", "I"), NOTATION_XIANGQI_WXF);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_THAI_SAN", "I"), NOTATION_THAI_SAN);
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "NOTATION_THAI_LAN", "I"), NOTATION_THAI_LAN);

    //validation
    env->SetStaticIntField(cls, env->GetStaticFieldID(cls, "FEN_OK", "I"), FEN::FEN_OK);


}